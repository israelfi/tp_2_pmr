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
import matplotlib.pyplot as plt



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
        """
        Gets the robot odometry (position and orientation)
        """
        self.robot_pos[0] = data.pose.pose.position.x
        self.robot_pos[1] = data.pose.pose.position.y

        x_q = data.pose.pose.orientation.x
        y_q = data.pose.pose.orientation.y
        z_q = data.pose.pose.orientation.z
        w_q = data.pose.pose.orientation.w
        euler = euler_from_quaternion([x_q, y_q, z_q, w_q])

        self.robot_ori = euler[2]

    def sensor_callback(self, data):
        """
        Reads the sensor data and finds the coordinates of each reading in the world frame
        """
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

    def get_local_min(self):
        local_min = []
        lidar_raw = self.lidar_raw
        for i in range(1, len(lidar_raw)-1):
            if(self.l_min < lidar_raw[i] < self.l_max):
                if(lidar_raw[i-1] > lidar_raw[i] and lidar_raw[i+1] > lidar_raw[i]):
                    local_min.append((lidar_raw[i], i))
        return local_min


    def dist_obst(self, px, py):
        """
        Returns: The distance between the robot and the point (px, py)
        """
        d = sqrt((px-self.robot_pos[0])**2 + (py-self.robot_pos[1])**2)
        return d

    def dist(self, px1, py1, px2, py2):
        """
        Returns: The distance between the point (px1, py1) and (px2, py2)
        """
        d = sqrt((px1-px2)**2 + (py1-py2)**2)
        return d

    def find_endpoints(self):
        end_ids = []
        cont = 0
        for i in range(1, len(self.lidar_raw)-1):
            if(self.l_min < self.lidar_raw[i] < self.l_max):
                if(self.lidar_raw[i+1] >= self.l_max):
                    end_ids.append(i)
                elif(self.lidar_raw[i-1] >= self.l_max):
                    end_ids.append(i)

        return end_ids

    def min_dist(self):
        """
        Returns: A list of tuples with the laser data ordered from the shortest reading to the farest one
        with the angle regarding the measurement (in the laser frame)

        Output format: [(distance, angle), (distance, angle), ..., (distance, angle)]
        """
        s = []
        alfa = []
        cont = 0
        aux_list = []
        for i in self.lidar_raw:
            a = (i, cont)
            aux_list.append(a)
            cont += 1
        aux_list.sort()

        for j in aux_list[:40]:
            s.append(j[0])
            alfa.append(j[1])

        return s, alfa

    def nearest_obstacles(self):
        s = []
        alfa = []
        cont = 0
        aux_list = []
        for i in self.lidar_raw:
            a = (i, cont)
            aux_list.append(a)
            cont += 1
        aux_list.sort()

        for j in aux_list[:10]:
            s.append(j[0])
            alfa.append(j[1])

        print(s)
        return s, alfa

    def rotate(self, s, alfa, obst_detec=1.0):
        """
        Rotate the robot
        """
        K = 1.0
        Ux_ = (2.0/pi)*atan(K*(s- obst_detec))  # Normal 
        Uy_ = sqrt(1.0-Ux_**2) # Tangent

        Ux = Ux_*cos(np.deg2rad(alfa-180)+self.robot_ori) - Uy_*sin(np.deg2rad(alfa-180)+self.robot_ori)
        Uy = Ux_*sin(np.deg2rad(alfa-180)+self.robot_ori) + Uy_*cos(np.deg2rad(alfa-180)+self.robot_ori)

        _, self.vel_msg.angular.z = self.controlador.feedback_linearization(Ux,Uy,self.robot_ori)
        self.vel_msg.linear.x = 0.5
        self.pub_cmd_vel.publish(self.vel_msg)

    def contourn_obst(self, s, alfa, obst_detec=1.0):
        """
        Contour a obstacle in a position of the laser (distance s and angle alfa) 
        """
        K = 1.0
        Ux_ = (2.0/pi)*atan(K*(s - obst_detec))  # Normal 
        Uy_ = sqrt(1.0-Ux_**2) # Tangent

        Ux = Ux_*cos(np.deg2rad(alfa-180)+self.robot_ori) - Uy_*sin(np.deg2rad(alfa-180)+self.robot_ori)
        Uy = Ux_*sin(np.deg2rad(alfa-180)+self.robot_ori) + Uy_*cos(np.deg2rad(alfa-180)+self.robot_ori)

        self.vel_msg.linear.x, self.vel_msg.angular.z = self.controlador.feedback_linearization(Ux,Uy,self.robot_ori)
        self.pub_cmd_vel.publish(self.vel_msg)

    def equidistant_obs(self, s, alfa, obst_detec=1.0):
        """
        Moves away form an obstacle
        """
        K = 0.25
        Ux_ = (2.0/pi)*atan(K*(s- obst_detec))  # Normal 
        Uy_ = 0.0

        Ux = Ux_*cos(np.deg2rad(alfa-180)+self.robot_ori) - Uy_*sin(np.deg2rad(alfa-180)+self.robot_ori)
        Uy = Ux_*sin(np.deg2rad(alfa-180)+self.robot_ori) + Uy_*cos(np.deg2rad(alfa-180)+self.robot_ori)

        self.vel_msg.linear.x, self.vel_msg.angular.z = self.controlador.feedback_linearization(Ux,Uy,self.robot_ori)
        self.pub_cmd_vel.publish(self.vel_msg)

    def dist_vec(self, alfa):
        """
        Calculates the distance between the items in a circular list ([0-360])
        """
        d = []
        for i in range(1, len(alfa)):
            dist_1 = (alfa[i] - alfa[i-1]) % 360
            dist_2 = (alfa[i-1] - alfa[i]) % 360
            d.append(min(dist_1, dist_2))

        return d
    
    def follow_target(self, px, py):
        """
        Makes the robot go to the point (px, py)
        """
        self.vel_msg.linear.x, self.vel_msg.angular.z = self.controlador.control([px,py], self.robot_pos, self.robot_ori)

        self.pub_cmd_vel.publish(self.vel_msg)

    def pot_rep(self, alfa1, alfa2=0):
        K = 2.0
        D_safe = 10.0
        pos = []
        maior = max(alfa1, alfa2)
        menor = min(alfa1, alfa2)
        alfa1 = maior
        alfa2 = menor
        pos.append(self.lidar_x[alfa1])
        pos.append(self.lidar_y[alfa1])

        pos.append(self.lidar_x[alfa2])
        pos.append(self.lidar_y[alfa2])

        alfa_rad = abs(atan2((pos[3] - pos[1]), (pos[2] - pos[0]))) #+ pi/2
    
        alfa = alfa1 + alfa2 #+ 90

        if alfa > 180:
            alfa -= 180

        grad_x = - cos(alfa*pi/180 + self.robot_ori)
        grad_y = - sin(alfa*pi/180 + self.robot_ori)

        Ux = K * grad_x
        Uy = K * grad_y

        self.vel_msg.linear.x, self.vel_msg.angular.z = self.controlador.feedback_linearization(Ux,Uy,self.robot_ori)
        self.pub_cmd_vel.publish(self.vel_msg)

    def pot_rep_obs(self, alfa1):
        K = 2.0
        D_safe = 10.0
        pos = []

        pos.append(self.lidar_x[alfa1])
        pos.append(self.lidar_y[alfa1])

        alfa = alfa1

        grad_x = cos(alfa*pi/180 + self.robot_ori)
        grad_y = sin(alfa*pi/180 + self.robot_ori)

        Ux = K * grad_x
        Uy = K * grad_y
       

        self.vel_msg.linear.x, self.vel_msg.angular.z = self.controlador.feedback_linearization(Ux,Uy,self.robot_ori)
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
                s, alfa = robot.min_dist()
                prev_alfa = alfa[0]

        # Go to a equidistant point between obstacles
        if (stage == 1):
            # Detect the nearest obstacles
            s, alfa = robot.min_dist()
            # robot.equidistant_obs(s[0], alfa[0], 10) # Moves away from the nearest obstacle
            robot.pot_rep_obs(alfa[0])

            # print(len(robot.get_local_min()), robot.get_local_min())

            local_min = robot.get_local_min()
            local_min.sort()
            print(local_min, "- 1")

            if (len(local_min) > 1):
                if (abs(local_min[0][0] - local_min[1][0]) < 0.20):
                    stage = 2
        
        # Equidistant to two obstacles
        if (stage == 2):
            s, alfa = robot.min_dist()
            robot.rotate(s[0], alfa[0], s[0])
            # robot.contourn_obst(s[0], alfa[0], s[0])

            local_min = robot.get_local_min()
            local_min.sort()
            print(local_min, "- 2")

            # Check if the nearest laser reading are from the same obstacle, if so, the robot needs to get
            # away from it
            if (len(local_min) > 1):
                robot.pot_rep(local_min[0][1], local_min[1][1])
                if (abs(local_min[0][0] - local_min[1][0]) > 0.20):
                    stage = 1
            else:
                robot.pot_rep(local_min[0][1])



        # Meetpoint (equidistant to three obstacles) - como identificar?
        if (stage == 3):
            pass

        rate.sleep()


if __name__ == '__main__':
    try:
        explore()
    except rospy.ROSInterruptException:
        pass