#!/usr/bin/env python
import rospy
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from math import cos, sin, pi, sqrt, atan2, atan
from sensor_msgs.msg import PointCloud2, LaserScan
import laser_geometry.laser_geometry as lg
import sensor_msgs.point_cloud2 as pc2
import numpy as np


class Robot:

    def __init__(self):
        self.robot_pos = [0, 0]
        self.robot_vel = [0, 0]
        self.robot_ori = 0

        self.lidar_raw = []
        self.lidar_x = [0] * 360
        self.lidar_y = [0] * 360
        self.l_max = 0
        self.l_min = 0
        self.vel_msg = Twist()

        self.controlador = Control()

        rospy.init_node("Voronoi", anonymous=True)
        rospy.Subscriber('/base_pose_ground_truth', Odometry, self.callback_robot_odom)
        rospy.Subscriber('/base_scan', LaserScan, self.sensor_callback)
        self.pub_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    def callback_robot_odom(self,data):
        self.robot_pos[0] = data.pose.pose.position.x
        self.robot_pos[1] = data.pose.pose.position.y

        x_q = data.pose.pose.orientation.x
        y_q = data.pose.pose.orientation.y
        z_q = data.pose.pose.orientation.z
        w_q = data.pose.pose.orientation.w
        euler = euler_from_quaternion([x_q, y_q, z_q, w_q])

        self.robot_ori = euler[2]

    def sensor_callback(self, data):
        self.lidar_raw  = data.ranges
        self.l_max = data.range_max
        self.l_min = data.range_min

        alfa = 0
        for x in self.lidar_raw:
            sx = (cos(self.robot_ori)*(x*cos(np.deg2rad(alfa-180))) - sin(self.robot_ori)*(x*sin(np.deg2rad(alfa-180)))) +self.robot_pos[0]
            sy = (sin(self.robot_ori)*(x*cos(np.deg2rad(alfa-180))) + cos(self.robot_ori)*(x*sin(np.deg2rad(alfa-180)))) +self.robot_pos[1]
            self.lidar_x[alfa]= sx
            self.lidar_y[alfa] = sy
            alfa = alfa+1

    def dist_obst(self, px, py):
        d = sqrt((px-self.robot_pos[0])**2 + (py-self.robot_pos[1])**2)
        return d

    def min_dist(self):
        s = []
        alfa = []
        cont = 0
        aux_list = []
        for i in self.lidar_raw:
            a = (i, cont)
            aux_list.append(a)
            cont += 1
        aux_list.sort()

        for j in aux_list[:3]:
            s.append(j[0])
            alfa.append(j[1])

        print(s)
        return s, alfa

    def contourn_obst(self, s, alfa, obst_detec=1.0):
        K = 1.0
        Ux_ = (2.0/pi)*atan(K*(s- obst_detec))  # Normal 
        Uy_ = sqrt(1.0-Ux_**2) # Tangent

        Ux = Ux_*cos(np.deg2rad(alfa-180)+self.robot_ori) - Uy_*sin(np.deg2rad(alfa-180)+self.robot_ori)
        Uy = Ux_*sin(np.deg2rad(alfa-180)+self.robot_ori) + Uy_*cos(np.deg2rad(alfa-180)+self.robot_ori)

        self.vel_msg.linear.x, self.vel_msg.angular.z = self.controlador.feedback_linearization(Ux,Uy,self.robot_ori)
        self.pub_cmd_vel.publish(self.vel_msg)

    def equidistant_obs(self, s, alfa, obst_detec=1.0):
        K = 1.0
        Ux_ = (2.0/pi)*atan(K*(s- obst_detec))  # Normal 
        Uy_ = 0.0

        Ux = Ux_*cos(np.deg2rad(alfa-180)+self.robot_ori) - Uy_*sin(np.deg2rad(alfa-180)+self.robot_ori)
        Uy = Ux_*sin(np.deg2rad(alfa-180)+self.robot_ori) + Uy_*cos(np.deg2rad(alfa-180)+self.robot_ori)

        self.vel_msg.linear.x, self.vel_msg.angular.z = self.controlador.feedback_linearization(Ux,Uy,self.robot_ori)
        self.pub_cmd_vel.publish(self.vel_msg)
    
    def follow_target(self, px, py):
        self.vel_msg.linear.x, self.vel_msg.angular.z = self.controlador.control([px,py], self.robot_pos, self.robot_ori)

        self.pub_cmd_vel.publish(self.vel_msg)


class Control:

    def __init__(self):
        self.d = 0.2
        self.k = 1

    def control(self,pos_curve, pos_robot, theta):
        """
        Entradas:
        pos_curve: Vetor contendo a posicao x,y de referencia da curva
        pos_robot: Posicao do robo no mundo
        Retorno:
        Ux: Velocidade no eixo x do sistema de coordenadas inercial
        Uy: Velocidade no eixo y do sistema de coordenadas inercial
        """

        Ux = self.k * (pos_curve[0] - pos_robot[0])
        Uy = self.k * (pos_curve[1] - pos_robot[1])

        return self.feedback_linearization(Ux,Uy,theta)

    def feedback_linearization(self,Ux, Uy, theta_n):
        """
        Entradas: 
        Ux: Velocidade no eixo x do sistema de coordenadas inercial
        Uy: Velocidade no eixo y do sistema de coordenadas inercial
        theta_n: orientacao do robo
        Retorno: 
        Vx: Velocidade no eixo x do robo 
        w: Velocidade angular do robo
        """

        vx = cos(theta_n) * Ux + sin(theta_n) * Uy
        w = -(sin(theta_n) * Ux)/ self.d  + (cos(theta_n) * Uy) / self.d 

        return vx, w


def explore():
    rospy.sleep(0.2)

    robot = Robot()
    rate = rospy.Rate(20)
    t_init = rospy.get_time()
    stage = 0

    while not rospy.is_shutdown():

        # Waiting laser data
        if(stage == 0):
            if(robot.lidar_raw):
                stage = 1

        # Go to a equidistant point between obstacles
        if (stage == 1):
            s, alfa = robot.min_dist()
            robot.contourn_obst(s[0], alfa[0])
        
        # Meetpoint
        if (stage == 2):
            pass
        # s, alfa = robot.min_dist()
        # print "Leitura mais prox:", s, "(m)\t", alfa, "(graus)"

        rate.sleep()


if __name__ == '__main__':
    try:
        explore()
    except rospy.ROSInterruptException:
        pass