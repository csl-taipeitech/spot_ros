import rospy
from sensor_msgs.msg import Joy
from std_srvs.srv import Trigger, SetBool, TriggerRequest, SetBoolRequest
from spot_msgs.srv import SetLocomotion, SetLocomotionRequest
from geometry_msgs.msg import Twist, Pose
from tf.transformations import quaternion_from_euler

class SpotControl:
    def __init__(self):
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.body_pub = rospy.Publisher('body_pose', Pose, queue_size=1)
        self.stair_mode = SetBoolRequest()
        self.stair_mode.data = False
        self.current_mode = ''
        self.linear_x_scale = 0.8
        self.linear_y_scale = 0.5
        self.angular_scale = 0.8
        
    def joy_callback(self, data: Joy):
        # Logi F710
        trig = TriggerRequest()
        # rospy.loginfo('current_mode: {}'.format(self.current_mode))
        if data.buttons[4] == 1 and data.buttons[5] == 1 and data.buttons[2] == 1:
            self.current_mode = 'cut power'
            self.estop_client.call(trig) # Estop
            return
        if data.buttons[4] == 1 and data.buttons[5] != 1 and data.buttons[0] == 1:
            srv = SetLocomotionRequest()
            srv.locomotion_mode = 8
            self.current_mode = 'Hop'
            self.locomotion_client.call(srv) # Hop
        if data.buttons[4] == 1 and data.buttons[5] != 1 and data.buttons[1] == 1:
            srv = SetLocomotionRequest()
            srv.locomotion_mode = 5
            self.current_mode = 'Amble'
            self.locomotion_client.call(srv) # Amble
        if data.buttons[4] == 1 and data.buttons[5] != 1 and data.buttons[2] == 1:
            srv = SetLocomotionRequest()
            srv.locomotion_mode = 4
            self.current_mode = 'Crawl'
            self.locomotion_client.call(srv) # Crawl
        if data.buttons[4] == 1 and data.buttons[5] != 1 and data.buttons[3] == 1:
            srv = SetLocomotionRequest()
            srv.locomotion_mode = 7
            self.current_mode = 'Jog'
            self.locomotion_client.call(srv) # Jog
        if data.buttons[4] != 1 and data.buttons[0] == 1:
            self.current_mode = 'Sit'
            self.sit_client.call(trig) # Sit
        if data.buttons[4] != 1 and data.buttons[1] == 1:
            srv = SetLocomotionRequest()
            srv.locomotion_mode = 1
            self.current_mode = 'Walk'
            self.locomotion_client.call(srv) # Walk
        if data.buttons[4] != 1 and data.buttons[2] == 1:
            self.current_mode = 'Stand'
            self.stand_client.call(trig) # Stand
        if data.buttons[4] != 1 and data.buttons[3] == 1 and self.stair_mode.data != True:
            self.stair_mode.data = True
            self.current_mode = 'Stair'
            self.stair_client.call(self.stair_mode) # Stair
        if data.buttons[9] == 1:
            self.current_mode = 'Auto'
            
        if self.current_mode == 'Hop' or self.current_mode == 'Jog' \
            or self.current_mode == 'Amble' or self.current_mode == 'Crawl' \
            or self.current_mode == 'Walk' or self.current_mode == 'Stair':
            # cmd_vel
            vel = Twist()
            vel.linear.x = data.axes[1]*self.linear_x_scale
            vel.linear.y = data.axes[0]*self.linear_y_scale
            vel.angular.z = data.axes[2]*self.angular_scale
            self.cmd_pub.publish(vel)
        
        if self.current_mode == 'Stand':
            pose = Pose()
            pose.position.x = 0
            pose.position.y = 0
            pose.position.z = data.axes[1]*0.2
            roll, pitch, yaw = -data.axes[0]*0.6, -data.axes[3]*0.6, data.axes[2]*0.6
            q = quaternion_from_euler(roll, pitch, yaw)
            pose.orientation.x = q[0]
            pose.orientation.y = q[1]
            pose.orientation.z = q[2]
            pose.orientation.w = q[3]
            self.body_pub.publish(pose)
        
        if self.current_mode != 'Stair' and self.stair_mode.data == True:
            self.stair_mode.data = False
            self.stair_client.call(self.stair_mode) # Stair
                        
    def main(self):
        rospy.init_node('spot_controller')
        
        rospy.Subscriber('joy', Joy, self.joy_callback)
        
        self.estop_client = rospy.ServiceProxy('estop/gentle', Trigger)
        self.stand_client = rospy.ServiceProxy('stand', Trigger)
        self.sit_client = rospy.ServiceProxy('sit', Trigger)
        self.stair_client = rospy.ServiceProxy('stair_mode', SetBool)
        self.locomotion_client = rospy.ServiceProxy('locomotion_mode', SetLocomotion)
        
        rospy.spin()