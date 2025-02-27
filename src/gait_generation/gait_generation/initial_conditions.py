import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from rosgraph_msgs.msg import Clock

class ApplyInitialForce(Node):
    def __init__(self):
        super().__init__('initial_conditions')

        # Simulation clock subscription
        self.clock_subscriber = self.create_subscription(Clock, '/clock', self.update_time, 5)
        
        # Initial Force Publisher
        self.force_publisher = self.create_publisher(Float64, '/quadruped/initial_condition_force', 10)
        
        # Set up clock
        # Use simulated time
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        
        # ROS Clock to use simulation time
        self.clock = self.get_clock()
        self.create_timer(1, self.timer_callback, clock=self.clock)
        
    def update_time(self, msg_in : Clock):
        self.sim_time = msg_in.clock.sec + msg_in.clock.nanosec * 1e-9

    def timer_callback(self):
        # Publish some big force to accelerate the quadruped body to regular velocity during the initalization phase
        msg_out = Float64()
        if self.sim_time <= 1:
            msg_out.data = 50.0
        else:
            msg_out.data = 0.0
        
        self.force_publisher.publish(msg_out)


def main(args=None):
    rclpy.init(args=args)
    node = ApplyInitialForce()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
