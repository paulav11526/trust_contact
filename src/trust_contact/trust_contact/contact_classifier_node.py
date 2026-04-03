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
        # Create Sample Training Data
        np.random.seed(42)
        X = []
        y = []

        for _ in range(1000):
            # Generate two contacts
            contact1 = [
                np.random.uniform(0, 15),   # force
                np.random.uniform(0, 2),      # start
                np.random.uniform(2, 5)      # end
            ]
            
            contact2 = [
                np.random.uniform(0, 15),
                np.random.uniform(0, 5),
                np.random.uniform(5, 10)
            ]
            
            matrix = np.array([contact1, contact2])
            features = self.extract_features(matrix)
            
            # Label rules (example logic)
            if features[4] < 0.5:
                label = "double tap"
            elif features[4] > 0.5:
                if features[2] > 2:
                    label = "long tap"
                else:
                    label = "continue"
            
            X.append(features)
            y.append(self.encode_label(label))

            X = pd.DataFrame(X, columns=[
                "force1", "force2", "duration1", "duration2", "gap", "force_diff"
            ])

            # Train Model
            model = RandomForestClassifier(n_estimators=100, random_state=42)
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
        # matrix shape: (2, 3)
        f1, t1_start, t1_end = matrix[0]
        f2, t2_start, t2_end = matrix[1]
        
        duration1 = t1_end - t1_start
        duration2 = t2_end - t2_start
        gap = t2_start - t1_end  # time between contacts
        force_diff = abs(f1 - f2)
        self.get_logger().info(f'Duration: {duration1}')
        self.get_logger().info(f'start: {t1_start}')
        self.get_logger().info(f'end: {t1_end}')
        
        return [f1, f2, duration1, duration2, gap, force_diff]

    # Encoding Function
    def encode_label(self,label):
            if label == "long tap":
                return [1, 0, 0]
            elif label == "double tap":
                return [0, 1, 0]
            else:  # "tap"
                return [0, 0, 1]

    # Prediction Function
    def predict_movement(self, matrix):
        features = self.extract_features(matrix)
        features_df = pd.DataFrame([features], columns=[
                "force1", "force2", "duration1", "duration2", "gap", "force_diff"
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

