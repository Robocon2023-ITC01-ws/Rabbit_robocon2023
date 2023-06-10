import rclpy

from rclpy.node import Node
from std_msgs.msg import String, Int8, Float32MultiArray
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

from rabbit_remote.rabbit_meca import RabbitModel


class PS4Remote(Node):

    def __init__(self):

        super().__init__('PS4_Node')

        self.rabbit_model = RabbitModel()

        self.subscribe_joy = self.create_subscription(
            Joy, "joy", self.subscribe_callback, 10
        )

        self.publisher_controls = self.create_publisher(
            Float32MultiArray, "input_controls", 10
        )

        self.pick_pub = self.create_publisher(Int8, 'pick_up', 10)
        
        self.twist_callback = self.create_timer(
            1/120, self.twist_teleop
        )

        self.control_publisher = self.create_publisher(String, "controller_state", 10)

        self.axes_list = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.button_list = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.coef_vx = 0.0
        self.coef_vy = 0.0
        self.coef_vth = 0.0
        self.coef_w1 = 0.0
        self.coef_w2 = 0.0

        self.a = 0.0
        self.b = 0.0
        self.c = 0.0

        self.cnt = 0
       

    def subscribe_callback(self, joy):
        
        self.axes_list = joy.axes
        self.button_list = joy.buttons
        msg = String()
        pick_msg = Int8()
        ################################### Switch state controller #################################
        if (self.button_list[9] == 1) and self.button_list[3] == 1 and self.cnt == 0:
            msg.data = "auto"
            self.control_publisher.publish(msg)
            print("auto")
            self.cnt = 1
        elif (self.button_list[9] == 1) and self.button_list[3] == 0 and self.cnt == 1:
            msg.data = "manual"
            self.control_publisher.publish(msg)
            print("manual")
            self.cnt = 0

        elif (self.button_list[4] == 1):
            pick_msg.data = 0
            self.pick_pub.publish(pick_msg)

        elif (self.button_list[5] == 1):
            pick_msg.data = 1
            self.pick_pub.publish(pick_msg)

         
        
    
    def twist_teleop(self):
        con_msg = Float32MultiArray()

        self.coef_vx = self.axes_list[1]
        self.coef_vy = self.axes_list[0]
        
        self.coef_vth =  self.axes_list[3]


        if self.button_list[2] == 1:
            self.a += 0.01
            self.b += 0.01
            self.c += 0.01
        elif self.button_list[0] == 1:
            self.a -= 0.01
            self.b -= 0.01
            self.c -= 0.01

        elif self.button_list[1] == 1:
            self.coef_vx = 0.0
            self.coef_vy = 0.0
            self.coef_vth = 0.0
            self.a = 0.0
            self.b = 0.0
            self.c = 0.0

        vx = self.coef_vx*self.a
        vy = self.coef_vy*self.b
        vyaw = self.coef_vth*self.c

        inv_vec = self.rabbit_model.inverse_kinematic(vx, vy, vyaw, 0.0, "numpy")

        con_msg.data = [float(inv_vec[0]), float(inv_vec[1]), float(inv_vec[2]), float(inv_vec[3])]

        self.publisher_controls.publish(con_msg)



def main(args=None):

    rclpy.init(args=args)

    node = PS4Remote()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()
