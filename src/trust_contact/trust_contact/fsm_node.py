import rclpy
from rclpy.node import Node
from enum import Enum
from std_msgs.msg import String
from sensor_msgs.msg import JointState

import numpy as np

# -------------------------
# Define States
# -------------------------
class State(Enum):
    STOPPED = 0
    AT_BIN_A = 1
    AT_BIN_B = 2
    MOVING_TO_A = 3
    MOVING_TO_B = 4


# -------------------------
# Define Events
# -------------------------
class Event(Enum):
    LONG_TAP = 0        # Stop
    DOUBLE_TAP = 1      # Change target
    SINGLE_TAP = 2       # Resume


class FSMNode(Node):

    def __init__(self):
        super().__init__('fsm_node')
        self.create_subscription(String, 'contact_events', self.event_callback, 10) # subscribing to topic that outputs long events
        self.create_subscription(JointState, 'joint_states', self.joint_state_callback ,10 ) 
        self.publisher = self.create_publisher(JointState, 'q_target', 10)

        # Define joint targets 
        self.BIN_A_q = [-0.54, -0.278518, -0.150196, -1.78548, -0.000276461, 2.16789, -0.0800078]
        self.BIN_B_q = [0.579967, -0.278543, -0.150242, -1.78547, -0.000275622, 2.1679, -0.0800071]
        self.STOP_q = None

        self.current_q = np.zeros(7)
        # Initial state
        self.state = State.STOPPED

        # Event flag
        self.event = None

        # Actions
        self.TARGET = True       # A - True, B- False
        self.USE_SPEED = True    # use modified speed
        self.IN_MOTION = False   # Robot is in motion
        self.GP_REACHED = False  # Goal position reached


        # Timer to run FSM loop
        self.timer = self.create_timer(0.1, self.fsm_loop)

        self.get_logger().info("FSM Node started")

    def joint_state_callback(self, msg: JointState):
        self.current_q = np.array(msg.position[:7])
    
    def event_callback(self, msg: String):
        if msg == 'Long_Tap':
            self.event = Event.LONG_TAP
        elif msg == "Double_Tap":
            self.event = Event.DOUBLE_TAP
        else:
            self.event = Event.SINGLE_TAP
        
        self.get_logger().info('Event received: {self.event}')

    # -------------------------
    # FSM Loop
    # -------------------------
    def fsm_loop(self):
        self.publish_target()

        if self.event is None:
            # check for "arrival" if robot is moving
            if self.state == State.MOVING_TO_A:
                if np.linalg.norm(self.current_q - self.BIN_A_q) < 0.05:
                    self.state = State.AT_BIN_A
                    self.get_logger().info('Arrived at A')
            elif self.state == State.MOVING_TO_B:
                if np.linalg.norm(self.current_q - self.BIN_B_q) < 0.05:
                    self.state = State.AT_BIN_B
                    self.get_logger().info('Arrived at B')
            return

        # Handle transitions
        if self.state == State.STOPPED:    # ------------------------------------------ STOPPED

            if self.event == Event.LONG_TAP: # E1
                self.state = State.STOPPED
                self.get_logger().info("STOPPED")
            
            elif (self.event == Event.SINGLE_TAP): # E3
                if (self.TARGET == True) & (self.GP_REACHED == True): # E3 T GP
                    self.state = State.AT_BIN_A
                    self.get_logger().info("Continue: STOPPED -> AT_BIN_A")
                    
            
                elif (self.TARGET == False) & (self.GP_REACHED == True): # E3 \T GP
                    self.state = State.AT_BIN_B
                    self.get_logger().info("Continue: STOPPED -> AT_BIN_B")

                elif (self.TARGET == True) & (self.GP_REACHED == False): # E3 T \GP
                    self.state = State.MOVING_TO_A
                    self.get_logger().info("Continue: STOPPED -> MOVING_TO_A")
                    
                elif (self.TARGET == False) & (self.GP_REACHED == False): # E3 \T \GP
                    self.state = State.MOVING_TO_A
                    self.get_logger().info("Continue: STOPPED -> MOVING_TO_A")
                
            elif self.event == Event.DOUBLE_TAP: # E2
                if  self.TARGET == True:
                    self.state = State.MOVING_TO_B 
                    self.get_logger().info("Transition: STOPPED -> MOVING_TO_B")
                elif self.TARGET == False:
                    self.state = State.MOVING_TO_A
                    self.get_logger().info("Transition: STOPPED -> MOVING_TO_A")

        elif self.state == State.AT_BIN_A:  # ------------------------------------------ AT BIN A
            self.TARGET = True
            self.USE_SPEED = False
            self.IN_MOTION = False
            self.GP_REACHED = True

            if self.event == Event.LONG_TAP: # E1
                self.state = State.STOPPED
                self.get_logger().info("Transition: AT_BIN_A -> STOPPED")

            elif self.event == Event.DOUBLE_TAP: # E2 
                self.state = State.MOVING_TO_B
                self.get_logger().info("Transition: AT_BIN_A -> MOVING_TO_B")

            elif self.event == Event.SINGLE_TAP: # E3
                self.state = State.AT_BIN_A
                self.get_logger().info("AT_BIN_A: Continue")

        elif self.state == State.AT_BIN_B:  # ------------------------------------------ AT BIN B
            self.TARGET = False
            self.USE_SPEED = False
            self.IN_MOTION = False
            self.GP_REACHED = True

            if self.event == Event.LONG_TAP: # E1
                self.state = State.STOPPED
                self.get_logger().info("Transition: AT_BIN_B -> STOPPED")

            elif self.event == Event.DOUBLE_TAP: # E2
                self.state = State.MOVING_TO_A
                self.get_logger().info("Transition: AT_BIN_B -> MOVING_TO_A")

            elif self.event == Event.SINGLE_TAP: # E3
                self.state = State.AT_BIN_B
                self.get_logger().info("AT_BIN_B: Continue")

        elif self.state == State.MOVING_TO_A:  # ------------------------------------------ MOVING TO A
            self.TARGET = True
            self.USE_SPEED = True
            self.IN_MOTION = True
            self.GP_REACHED = False

            if self.event == Event.LONG_TAP: # E1
                self.state = State.STOPPED
                self.get_logger().info("Transition: MOVING_TO_A -> STOPPED")

            elif self.event == Event.DOUBLE_TAP: # E2
                self.TARGET = False
                self.state = State.MOVING_TO_B
                self.get_logger().info("Transition: MOVING_TO_A -> MOVING_TO_B")
            
            elif self.IN_MOTION == False:
                self.state == State.AT_BIN_A
                self.get_logger().info("Transition: MOVING_TO_A -> AT_BIN_A")

            elif self.event == Event.SINGLE_TAP:
                self.state = State.MOVING_TO_A
                self.get_logger().info("Continue MOVING_TO_A")

        elif self.state == State.MOVING_TO_B:  # ------------------------------------------ MOVING TO B
            self.TARGET = False
            self.USE_SPEED = True
            self.IN_MOTION = True
            self.GP_REACHED = False

            if self.event == Event.LONG_TAP: # E1
                self.state = State.STOPPED
                self.get_logger().info("Transition: MOVING_TO_B -> STOPPED")

            elif self.event == Event.DOUBLE_TAP: # E2
                self.state = State.MOVING_TO_A 
                self.get_logger().info("Transition: MOVING_TO_B -> MOVING_TO_A")
            
            elif self.IN_MOTION == False:
                self.state == State.AT_BIN_B
                self.get_logger().info("Transition: MOVING_TO_B -> AT_BIN_B")

            elif self.event == Event.SINGLE_TAP:
                self.state = State.MOVING_TO_B
                self.get_logger().info("Continue MOVING_TO_B")
            

        # Clear event after processing
        self.event = None

    def publish_target(self):
        msg = JointState()
        if self.state == State.STOPPED:
            msg.position = self.current_q.tolist() # send exact position as goal position
        elif self.state == State.MOVING_TO_A or self.state == State.AT_BIN_A:
            msg.position = self.BIN_A_q
        elif self.state == State.MOVING_TO_B or self.state == State.AT_BIN_B:
            msg.position = self.BIN_B_q
        
        self.publisher.publish(msg)
        self.get_logger().info(f'Target: {self.state}')


# -------------------------
# Main
# -------------------------
def main(args=None):

    rclpy.init(args=args)

    node = FSMNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
