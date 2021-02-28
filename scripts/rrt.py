#!/usr/bin/env python
import rospy
import rospkg
from tf.transformations import euler_from_quaternion

# ros-msgs
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PointStamped

# python
import matplotlib.image as img
import numpy as np


class rrt():
    def __init__(self, start, goal, max_samples):
        self.test = 0
        (x,y) = start
        self.x = []
        self.y = []
        self.x.append(x)
        self.y.append(y)
        self.parent = []




##    Rotina callback para a obtencao da pose do robo
def callback_pose(data):
    global robot_states

    robot_states[0] = data.pose.pose.position.x  # posicao 'x' do robo no mundo 
    robot_states[1] = data.pose.pose.position.y  # posicao 'y' do robo no mundo 

    x_q = data.pose.pose.orientation.x
    y_q = data.pose.pose.orientation.y
    z_q = data.pose.pose.orientation.z
    w_q = data.pose.pose.orientation.w
    euler = euler_from_quaternion([x_q, y_q, z_q, w_q])

    robot_states[2] = euler[2]  # orientacao do robo no mundo 
            
    return


def callback_goalPoint(data):
    global goal
    (goal[0],goal[1]) = [data.point.x, data.point.y]
    print(goal)



########################################
'''           Discret Map            '''
########################################
def map(map_name):
    rospack = rospkg.RosPack()
    path = rospack.get_path('tp_1')
    image_path = path + '/worlds/' + map_name
    image = img.imread(image_path)
    image.setflags(write=1)

    M = np.zeros((len(image),len(image)))
    for i in range(len(image)):
        for j in range(len(image)):
            if(image[i,j,0] == 255 and image[i,j,1] == 255 and image[i,j,2] == 255):
                M[i,j] = 0
            else:
                M[i,j] = 1

    return M


########################################
'''           Main Function          '''
########################################
def run():
    global robot_states, goal

    # states - x,y, theta
    robot_states = [0.0, 0.0, 0.0]

    ## ROS STUFFS
    rospy.init_node("rrt", anonymous=True)

    # Publishers
    pub_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    # Subscribers
    rospy.Subscriber('/base_pose_ground_truth', Odometry, callback_pose)
    rospy.Subscriber('/clicked_point', PointStamped, callback_goalPoint)

    # routine frequency
    rate = rospy.Rate(20)

    ####### Get Map
    M = map('map_obstacle2.bmp')

    ####### RRT - class
    # Start point
    start = [1.0, 1.0]
    goal = [5.0, 5.0]
    max_samples = 100
    planner = rrt(start, goal, max_samples)

    while not rospy.is_shutdown():
    	# print("Robot pose: x = %f, y = %f, yaw = %f\n" % (robot_states[0],robot_states[1],robot_states[2]))
    	rate.sleep()



########################################
'''            Main Routine          '''
########################################
if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
