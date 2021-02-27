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