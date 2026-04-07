import random
import numpy as np
from sklearn.linear_model import LinearRegression

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float32MultiArray


class SpeedPredictor(Node):
    def __init__(self):
        super().__init__('speed_predictor_node')

        self.max_speed = 2.0
        self.alpha = 0.5
        self.beta = 0.25
        self.n_samples = 1000

        self.model = self.train_model()

        self.subscription = self.create_subscription(Float32MultiArray, 'contact_parameters', self.input_callback, 10) # get C1 and C2
        self.publisher = self.create_publisher(Float64, 'predicted_speed', 10) # publish new speed

        self.get_logger().info('Speed predictor node started.')

    def contact_type_data(self, contact_type: float) -> float:
        """
        Generates noisy contact-based speed contribution for synthetic training.
        """
        if contact_type >= 1:
            if random.random() > 0.75:
                speed = 0.8
            else:
                speed = 0.5
        else:
            if random.random() > 0.75:
                speed = 0.2
            else:
                speed = 0.5
        return speed

    def trust_parameter_data(self, trust_p: float) -> float:
        """
        Generates trust-based speed contribution for synthetic training.
        """
        return float(np.square(trust_p))

    def train_model(self) -> LinearRegression:
        random.seed(42)
        np.random.seed(42)

        # C1[:,0] = raw trust parameter
        # C1[:,1] = noisy/derived trust contribution to speed
        C1 = np.zeros((self.n_samples, 2))

        # C2[:,0] = raw contact type input
        # C2[:,1] = noisy/derived contact contribution to speed
        C2 = np.zeros((self.n_samples, 2))

        speed_true = np.zeros(self.n_samples)

        for i in range(self.n_samples):
            C1[i, 0] = random.random()
            C1[i, 1] = self.trust_parameter_data(C1[i, 0])

            C2[i, 0] = random.choice([0, 1])
            C2[i, 1] = self.contact_type_data(C2[i, 0])

            speed_true[i] = (
                self.alpha * C1[i, 1] + self.beta * C2[i, 1]
            ) * self.max_speed

        # Train on RAW inputs
        X = np.column_stack((C1[:, 0], C2[:, 0]))

        model = LinearRegression()
        model.fit(X, speed_true)

        self.get_logger().info(
            f'Trained model: coef={model.coef_}, intercept={model.intercept_}'
        )
        return model

    def input_callback(self, msg: Float32MultiArray) -> None:
        if len(msg.data) < 2:
            self.get_logger().warning('Expected Float32MultiArray with [C1, C2]')
            return

        c1 = float(msg.data[0])
        c2 = float(msg.data[1])

        # Optional sanity clamp
        c1 = max(0.0, min(1.0, c1))

        X_new = np.array([[c1, c2]], dtype=float)
        predicted_speed = float(self.model.predict(X_new)[0])

        # Clamp final speed to a sensible range
        predicted_speed = max(0.0, min(self.max_speed, predicted_speed))

        out_msg = Float64()
        out_msg.data = predicted_speed
        self.publisher.publish(out_msg)

        self.get_logger().info(
            f'Received C1={c1:.3f}, C2={c2:.1f} -> Predicted speed={predicted_speed:.3f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = SpeedPredictor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



