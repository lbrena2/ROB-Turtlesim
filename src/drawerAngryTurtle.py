#!/usr/bin/env python
#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt
from turtlesim.srv import Spawn
from turtlesim.srv import SetPen
from turtlesim.srv import TeleportAbsolute
from turtlesim.srv import TeleportRelative
PI = 3.1415926535897





class TurtleBot:

    def __init__(self):
        # Creates a node with name 'turtlebot_controller' and make sure it is a
        # unique node (using anonymous=True).
        rospy.init_node('turtlebot_controller', anonymous=True)

        
        # Publisher which will publish to the topic '/turtle1/cmd_vel'.
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel',
                                                  Twist, queue_size=10)

        # A subscriber to the topic '/turtle1/pose'. self.update_pose is called
        # when a message of type Pose is received.
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose',
                                                Pose, self.update_pose)

        spawn = rospy.ServiceProxy('/spawn', Spawn)
        spawn(0, 0, -1, 'turtle2')

        self.velocity_publisher2 = rospy.Publisher('/turtle2/cmd_vel',
                                                  Twist, queue_size=10)
        self.pose_subscriber2 = rospy.Subscriber('/turtle2/pose',
                                                Pose, self.update_pose2)
        ## possible states:{ANGRY, WRITING, RETURNING}
        self.turle1_state = 'WRITING'
        self.pose = Pose()
        self.pose2 = Pose()

        self.rate = rospy.Rate(10)

    def update_pose(self, data):
        """Callback function which is called when a new message of type Pose is
        received by the subscriber."""
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)

    def update_pose2(self, data):
        """Callback function which is called when a new message of type Pose is
        received by the subscriber."""
        self.pose2 = data
        self.pose2.x = round(self.pose2.x, 4)
        self.pose2.y = round(self.pose2.y, 4)

    def euclidean_distance(self, goal_pose):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((goal_pose.x - self.pose.x), 2) +
                    pow((goal_pose.y - self.pose.y), 2))

    def linear_vel(self, goal_pose, constant=1):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return constant * self.euclidean_distance(goal_pose)

    def steering_angle(self, goal_pose):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

    def angular_vel(self, goal_pose, constant=1):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return constant * np.arctan2(np.sin(self.steering_angle(goal_pose) - self.pose.theta), np.cos(self.steering_angle(goal_pose) - self.pose.theta))
    
    def move2goal(self):
        set_pen = rospy.ServiceProxy('turtle1/set_pen', SetPen)        
        self.rotate_turtle(60, 180)
        
        set_pen(0,0,0,0,1)
        u = Pose(0.5, 5.5, 0, 0, 0)
        self.draw(u)
        self.rotate_turtle(30, 90)
        set_pen(0,0,0,0,0)
        u = Pose(0.5, 2.0, 0, 0, 0)
        self.draw(u)
        u = Pose(3.5, 1.5, 0, 0, 0)
        self.draw(u)
        u = Pose(3.5, 5.5, 0, 0, 0)
        self.draw(u)

        
        set_pen(0,0,0,0,1)
        s = Pose(7, 5.5, 0, 0, 0)
        self.draw(s)
        self.rotate_turtle(90, 270)
        set_pen(0,0,0,0,0)
        s = Pose(6, 5.5, 0, 0, 0)
        self.draw(s)
        s = Pose(5.5, 5, 0, 0, 0)
        self.draw(s)
        s = Pose(5.5, 4, 0, 0, 0)
        self.draw(s)
        s = Pose(6.0, 3.5, 0, 0, 0)
        self.draw(s)
        s = Pose(6.5, 3.5, 0, 0, 0)
        self.draw(s)
        s = Pose(7, 3.5, 0, 0, 0)
        self.draw(s)
        s = Pose(7.5, 2, 0, 0, 0)
        self.draw(s)
        s = Pose(4, 1, 0, 0, 0)
        self.draw(s)
        
        set_pen(0,0,0,0,1)
        i = Pose(9, 5.5, 0, 0, 0)
        self.draw(i)
        set_pen(0,0,0,0,0)
        self.rotate_turtle(90, 300)
        i = Pose(9, 1, 0, 0, 0)
        self.draw(i)
        
        

    def follow_turtle(self):
        set_pen = rospy.ServiceProxy('turtle1/set_pen', SetPen)
        while not rospy.is_shutdown() and self.turle1_state == 'ANGRY':
            print("Im following")
            vel_msg = Twist() 
            vel_msg2 = Twist() 
            # Linear velocity in the x-axis.
            vel_msg.linear.x = self.linear_vel(self.pose2)
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = self.angular_vel(self.pose2)
            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)
            if self.euclidean_distance(self.pose2) < 0.1:
                print("Im return")
                self.turle1_state = 'RETURNING'
                set_pen(0,0,0,0,1)
                u = Pose(0.5, 5.5, 0, 0, 0)
                self.draw(u)
                self.turle1_state = 'WRITING'
                self.move2goal()
            self.rate.sleep()
    
    def draw(self, goal_pose):

        while self.euclidean_distance(goal_pose) >= 0.1:
            vel_msg = Twist()
            #print("Im writing")
            if not self.turle1_state == 'RETURNING':
                self.turle1_state = 'WRITING'
            # Linear velocity in the x-axis.
            vel_msg.linear.x = 0.5*self.linear_vel(goal_pose)
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = self.angular_vel(goal_pose)

            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)
            if self.euclidean_distance(self.pose2) < 0.1 and self.turle1_state == 'WRITING':
                self.turle1_state = 'ANGRY'
                self.follow_turtle()
                self.move2goal()
            # Publish at the desired rate.
            self.rate.sleep()


    def rotate_turtle(self, speed, angle):
        vel_msg = Twist()
        #Converting from angles to radians
        angular_speed = speed*2*PI/360
        relative_angle = angle*2*PI/360

        #We wont use linear components
        vel_msg.linear.x=0
        vel_msg.linear.y=0
        vel_msg.linear.z=0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0

        vel_msg.angular.z = abs(angular_speed)
        # Setting the current time for distance calculus
        t0 = rospy.Time.now().to_sec()
        current_angle = 0

        while(current_angle < relative_angle):
            self.velocity_publisher.publish(vel_msg)
            t1 = rospy.Time.now().to_sec()
            current_angle = angular_speed*(t1-t0)

        


        

if __name__ == '__main__':
    try:
        x = TurtleBot()
        x.move2goal()
    except rospy.ROSInterruptException:
        pass