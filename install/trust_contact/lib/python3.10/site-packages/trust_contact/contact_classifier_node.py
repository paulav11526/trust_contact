import numpy as np
import pandas as pd
from sklearn.ensemble import RandomForestClassifier

import rclpy
from rclpy.node import Node
import numpy as np
import pandas as pd
from sklearn.ensemble import RandomForestClassifier
from messages.msg import ForceEvent  
from std_msgs.msg import String      

class ContactClassifier(Node):
    def __init__(self):
        super().__init__('contact_classifier_node')

        self.create_subscription(ForceEvent, 'force_data', self.force_listener_callback,10)
        self.publisher1 = self.create_publisher(String, 'contact_events', 10)

        # Setup model - training once on startup
        self.model = self.train()
        self.prediction = None

        self.get_logger().info('Classifier node started')

    def train(self):
        np.random.seed(42)

        X = []
        y = []

        n_samples = 3000

        for _ in range(n_samples):
            # choose intent class
            intent = np.random.choice(["single tap", "double tap", "long tap"])

            # --- First contact always exists ---
            force1 = np.random.uniform(1.0, 15.0)
            t1_start = np.random.uniform(0.0, 5.0)

            if intent == "single tap":
                duration1 = np.random.uniform(0.04, 0.35)
                t1_end = t1_start + duration1

                # no second contact
                force2 = 0.0
                t2_start = 0.0
                t2_end = 0.0

            elif intent == "long tap":
                duration1 = np.random.uniform(0.8, 3.0)
                t1_end = t1_start + duration1

                # no second contact
                force2 = 0.0
                t2_start = 0.0
                t2_end = 0.0

            else:  # double tap
                duration1 = np.random.uniform(0.04, 0.30)
                t1_end = t1_start + duration1

                gap = np.random.uniform(0.05, 0.45)

                force2 = np.random.uniform(1.0, 15.0)
                t2_start = t1_end + gap
                duration2 = np.random.uniform(0.04, 0.30)
                t2_end = t2_start + duration2

            # Add small measurement noise/jitter
            force1 = max(0.0, force1 + np.random.normal(0, 0.3))
            force2 = max(0.0, force2 + np.random.normal(0, 0.3))

            t1_start = max(0.0, t1_start + np.random.normal(0, 0.005))
            t1_end = max(t1_start, t1_end + np.random.normal(0, 0.005))

            if force2 > 0.0:
                t2_start = max(t1_end, t2_start + np.random.normal(0, 0.005))
                t2_end = max(t2_start, t2_end + np.random.normal(0, 0.005))

            matrix = np.array([
                [force1, t1_start, t1_end],
                [force2, t2_start, t2_end]
            ], dtype=float)

            features = self.extract_features(matrix)

            X.append(features)
            y.append(self.encode_label(intent))

        X = pd.DataFrame(X, columns=[
            "force1",
            "force2",
            "duration1",
            "duration2",
            "gap",
            "force_diff",
            "has_second_contact"
        ])

        model = RandomForestClassifier(
            n_estimators=200,
            random_state=42,
            max_depth=8,
            min_samples_split=5,
            min_samples_leaf=2
        )
        model.fit(X, y)

        return model

    def ros_time_to_float(self, ros_time):
        return ros_time.sec + (ros_time.nanosec / 1e9)

    def force_listener_callback(self, msg: ForceEvent):
        # convert message into 3x2 matrix
        t1_s = self.ros_time_to_float(msg.timestamp_1_start)
        t1_e = self.ros_time_to_float(msg.timestamp_1_end)
        t2_s = self.ros_time_to_float(msg.timestamp_2_start)
        t2_e = self.ros_time_to_float(msg.timestamp_2_end)

        matrix = np.array([
            [msg.force_magnitude_1, t1_s, t1_e],
            [msg.force_magnitude_2, t2_s, t2_e]
        ])
        self.get_logger().info(f"input data: {matrix}")
        self.prediction = self.predict_movement(matrix)
        self.publish_event()

    # Feature Extraction Function
    def extract_features(self, matrix):
        f1, t1_start, t1_end = matrix[0]
        f2, t2_start, t2_end = matrix[1]

        duration1 = max(0.0, t1_end - t1_start)

        has_second_contact = not (
            f2 == 0.0 and t2_start == 0.0 and t2_end == 0.0
        )

        if has_second_contact:
            duration2 = max(0.0, t2_end - t2_start)
            gap = max(0.0, t2_start - t1_end)
            force_diff = abs(f1 - f2)
        else:
            duration2 = 0.0
            gap = 0.0
            force_diff = 0.0

        return [f1,f2,duration1,duration2,gap,force_diff,float(has_second_contact)]

    # Encoding Function
    def encode_label(self,label):
            if label == "long tap":
                return [1, 0, 0]
            elif label == "double tap":
                return [0, 1, 0]
            else:  # "single tap"
                return [0, 0, 1]

    # Prediction Function
    def predict_movement(self, matrix):
        features = self.extract_features(matrix)
        features_df = pd.DataFrame([features], columns=[
                "force1", "force2", "duration1", "duration2", "gap", "force_diff", "has_second_contact",
            ])

        pred = self.model.predict(features_df)[0]
        #print(pred)
        return pred

    def publish_event(self):
        command = String()
        if np.array_equal(self.prediction, [1, 0, 0]):
            command.data = "Long_Tap" # Long tap
        elif np.array_equal(self.prediction, [0, 1, 0]):
            command.data = "Double_Tap" # Double tap
        else:
            command.data = "Single_Tap"

        self.publisher1.publish(command)
        self.get_logger().info(f"Intent classified: {command.data}")

def main(args=None):
    rclpy.init(args=args)
    node = ContactClassifier()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

