import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import String, Int8, Int8, Float32
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

from rabbit_remote.rabbit_meca import RabbitModel
import time


class PS4Remote(Node):

    def __init__(self):

        super().__init__('PS4_Node')

        self.rabbit_model = RabbitModel()

        self.subscribe_joy = self.create_subscription(
            Joy, "joy", self.subscribe_callback, 10
        )

        self.publisher_controls = self.create_publisher(
            Twist, "cmd_vel", 10
        )

        self.pick_pub = self.create_publisher(Int8, 'pick_up', 10)

        self.shooter_pub = self.create_publisher(Int8, 'shooter_command', 10)
        self.twist_callback = self.create_timer(
            1/120, self.twist_teleop
        )

        self.control_publisher = self.create_publisher(String, "controller_state", 10)

        self.retry_state = self.create_publisher(String, "retry_state", 10)

        self.command_publisher = self.create_publisher(String, "cmd_goal", 10)

        self.adjust_left = self.create_publisher(Float32, 'adjust_left', 10)
        self.adjust_right = self.create_publisher(Float32, 'adjust_right', 10)

        self.data_pub = self.create_publisher(Int8, 'data_save', 10)

        # self.shooter_timer = self.create_timer(1, self.shooter_callback)
        # self.pick_up_timer_2 = self.create_timer(1, self.pick_up_callback_2)
        # self.pick_up_timer_3 = self.create_timer(1, self.pick_up_callback_3)



        self.axes_list = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.button_list = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        self.coef_vx = 0.0
        self.coef_vy = 0.0
        self.coef_vth = 0.0
        self.coef_w1 = 0.0
        self.coef_w2 = 0.0

        self.a = 0.0
        self.b = 0.0
        self.c = 0.0

        self.cnt = 1

    # def shooter_callback(self):
    # 	sh_msg = Int8()
    # 	if(self.button_list[5]):
    #         sh_msg.data = self.button_list[5]
    #         self.shooter_pub.publish(sh_msg)

    # def pick_up_callback_2(self):
    # 	pick_msg = Int8()
    # 	if (self.axes_list[7] == -1.0):
    #         pick_msg.data = 2
    #         self.pick_pub.publish(pick_msg)

    # def pick_up_callback_3(self):
    # 	pick_msg = Int8()
    # 	if (self.axes_list[7] == -1.0):
    #         pick_msg.data = 3
    #         self.pick_pub.publish(pick_msg)




    def subscribe_callback(self, joy):
        pick_msg = Int8()
        self.axes_list = joy.axes
        self.button_list = joy.buttons
        msg = String()
        st_msg = String()
        sh_msg = Int8()
        pick_msg = Int8()
        adj_right = Float32()
        adj_left = Float32()
        save_msg = Int8()
        ################################### Switch state controller #################################
        if (self.button_list[8] == 1) and self.button_list[3] == 1 and self.cnt == 0:
            msg.data = "manual"
            self.control_publisher.publish(msg)
            print("manual")
            self.cnt = 1
        if (self.button_list[8] == 1) and self.button_list[3] == 0 and self.cnt == 1:
            msg.data = "auto"
            self.control_publisher.publish(msg)
            print("auto")
            self.cnt = 0

        if (self.axes_list[5] != 0):
            adj_right.data = self.axes_list[5]
            self.adjust_right.publish(adj_right)

        if (self.axes_list[2] != 0):
            adj_left.data = self.axes_list[2]
            self.adjust_left.publish(adj_left)

        if (self.button_list[3] == 0 and self.button_list[10] == 1):
            msg.data = "retry1"
            self.retry_state.publish(msg)
            print("Retry1")
        if (self.button_list[3] == 1 and self.button_list[10] == 1):
            msg.data = "retry2"
            self.retry_state.publish(msg)
            print("Retry2")


        if (self.axes_list[7] == 1.0):
            st_msg.data = "goal2"
            self.command_publisher.publish(st_msg)
        if (self.axes_list[6] == 1.0):
            st_msg.data = "goal3"
            self.command_publisher.publish(st_msg)
            # sh_msg.data = 0
            # self.shooter_pub.publish(sh_msg)

        if (self.button_list[5] == 1):
            pick_msg.data = 1
            self.pick_pub.publish(pick_msg)
        if (self.button_list[4] == 1):
            pick_msg.data = 4
            self.pick_pub.publish(pick_msg)

        if (self.axes_list[7] == -1.0):
            pick_msg.data = 2
            self.pick_pub.publish(pick_msg)

        if (self.axes_list[6] == -1.0):
            pick_msg.data = 3
            self.pick_pub.publish(pick_msg)


        if (self.button_list[9]):
            save_msg.data = self.button_list[9]
            self.data_pub.publish(save_msg)
            print("Worked")


        # if (self.button_list[4] == 1):
        #     pick_msg.data = self.button_list[4]
        #     self.pick_pub.publish(pick_msg)
        # else:
        #     sh_msg.data = 0
        #     self.shooter_pub.publish(sh_msg)
        #     save_msg.data = 0
        #     self.data_pub.publish(save_msg)
        #     pick_msg.data = 0
        #     self.pick_pub.publish(pick_msg)

        sh_msg.data = self.button_list[1]
        self.shooter_pub.publish(sh_msg)


    def twist_teleop(self):
        con_msg = Twist()

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

        con_msg.linear.x = vx
        con_msg.linear.y = vy
        con_msg.angular.z = vyaw


        self.publisher_controls.publish(con_msg)



def main(args=None):

    rclpy.init(args=args)

    node = PS4Remote()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()
