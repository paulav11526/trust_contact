import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Bool, String, Float64
from std_msgs.msg import Float64MultiArray
import numpy as np
import pandas as pd

class EvaluationLogger(Node):
    def __init__(self):
        super().__init__('evaluation_logger')

        self.trial_id = None
        self.event_id = None
        self.true_tap = None
        self.predicted_tap = None
        self.actual_action = None
        self.force_error = None

        self.event_active = False
        self.prev_event_active = False

        self.current_event = None
        self.rows = []
        # from simulation node
        self.create_subscription(Int32, '/eval/trial_id', self.trial_cb, 10)
        self.create_subscription(Int32, '/eval/event_id', self.event_cb, 10)
        self.create_subscription(String, '/eval/true_tap', self.true_tap_cb, 10)
        self.create_subscription(Float64MultiArray, '/observer/qfrc_applied', self.tau_ref_cb, 10)
        self.create_subscription(Bool, '/eval/event_active', self.event_active_cb, 10)
        # from fsm
        self.create_subscription(String, '/eval/actual_action', self.action_cb, 10)
        # from residual
        self.create_subscription(Float64, '/eval/force_error', self.force_error_cb, 10)
        self.create_subscription(Float64MultiArray, '/observer/residual', self.residual_cb, 10)
        # from RF node
        self.create_subscription(String, 'contact_events', self.pred_tap_cb, 10)
        
        

        self.latest_residual = None
        self.latest_tau_ref = None

    def trial_cb(self, msg):
        self.trial_id = msg.data

    def event_cb(self, msg):
        self.event_id = msg.data

    def true_tap_cb(self, msg):
        self.true_tap = msg.data

    def pred_tap_cb(self, msg):
        self.predicted_tap = msg.data
        if self.current_event is not None:
            self.current_event["predicted_tap"] = msg.data

    def action_cb(self, msg):
        self.actual_action = msg.data
        if self.current_event is not None:
            self.current_event["actual_action"] = msg.data

    def force_error_cb(self, msg):
        self.force_error = msg.data
        if self.current_event is not None:
            self.current_event["force_error"] = msg.data

    def residual_cb(self, msg):
        self.latest_residual = np.array(msg.data)
        if self.event_active and self.current_event is not None and self.latest_tau_ref is not None:
            self.current_event["residual_samples"].append(self.latest_residual.copy())
            self.current_event["tau_ref_samples"].append(self.latest_tau_ref.copy())

    def tau_ref_cb(self, msg):
        self.latest_tau_ref = np.array(msg.data)

    def event_active_cb(self, msg):
        self.prev_event_active = self.event_active
        self.event_active = msg.data

        if (not self.prev_event_active) and self.event_active:
            self.start_event()

        elif self.prev_event_active and (not self.event_active):
            self.end_event()

    def start_event(self):
        self.current_event = {
            "trial_id": self.trial_id,
            "event_id": self.event_id,
            "true_tap": self.true_tap,
            "predicted_tap": self.predicted_tap,
            "actual_action": self.actual_action,
            "force_error": self.force_error,
            "residual_samples": [],
            "tau_ref_samples": []
        }
        self.get_logger().info(f"Started event trial={self.trial_id}, event={self.event_id}")

    def expected_action_from_tap(self, tap):
        mapping = {
            "Single_Tap": "continue",
            "Double_Tap": "change_target",
            "Long_Tap": "stop"
        }
        return mapping.get(tap, "unknown")

    def compute_rmse(self, residual_samples, tau_ref_samples):
        if len(residual_samples) == 0 or len(tau_ref_samples) == 0:
            return np.nan
        r = np.array(residual_samples)
        tau = np.array(tau_ref_samples)
        return np.sqrt(np.mean(np.sum((r - tau)**2, axis=1)))

    def end_event(self):
        if self.current_event is None:
            return

        rmse = self.compute_rmse(
            self.current_event["residual_samples"],
            self.current_event["tau_ref_samples"]
        )

        true_tap = self.current_event["true_tap"]
        pred_tap = self.current_event["predicted_tap"]
        actual_action = self.current_event["actual_action"]
        expected_action = self.expected_action_from_tap(true_tap)

        row = {
            "trial_id": self.current_event["trial_id"],
            "event_id": self.current_event["event_id"],
            "true_tap": true_tap,
            "predicted_tap": pred_tap,
            "rf_correct": pred_tap == true_tap,
            "expected_action": expected_action,
            "actual_action": actual_action,
            "fsm_correct": actual_action == expected_action,
            "residual_rmse": rmse,
            "force_error_percent": self.current_event["force_error"]
        }

        self.rows.append(row)
        self.get_logger().info(f"Finished event: {row}")
        self.current_event = None

        if len(self.rows) == 15:
            self.save_results()

    def save_results(self):
        df = pd.DataFrame(self.rows)
        df.to_csv('trust_contact_event_metrics.csv', index=False)

        rf_accuracy = 100 * df["rf_correct"].mean()
        fsm_success = 100 * df["fsm_correct"].mean()
        residual_mean = df["residual_rmse"].mean()
        residual_std = df["residual_rmse"].std()
        force_mean = df["force_error_percent"].mean()
        force_std = df["force_error_percent"].std()

        with open('trust_contact_summary.txt', 'w') as f:
            f.write(f"RF accuracy: {rf_accuracy:.2f}%\n")
            f.write(f"FSM success rate: {fsm_success:.2f}%\n")
            f.write(f"Residual RMSE: {residual_mean:.4f} ± {residual_std:.4f}\n")
            f.write(f"Force error (%): {force_mean:.2f} ± {force_std:.2f}\n")

        self.get_logger().info("Saved evaluation results.")

def main(args=None):
    rclpy.init(args=args)
    node = EvaluationLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()