#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Software License Agreement (BSD)
#
# @author    Salman Omar Sohail <support@mybotshop.de>
# @copyright (c) 2025, MYBOTSHOP GmbH, Inc., All rights reserved.
#

#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Redistribution and use in source and binary forms, with or without
# modification, is not permitted without the express permission
# of MYBOTSHOP GmbH.

from math import sin, cos
import numpy as np

def column_vector(array):
    column_vector = array[np.newaxis].T
    return column_vector

def rotx(alpha):
    """
    Create a 3x3 rotation matrix about the x axis
    """
    rx = np.array([[1, 0,          0,         ],
                   [0, cos(alpha), -sin(alpha)],
                   [0, sin(alpha), cos(alpha) ]])

    return rx

def roty(beta):
    """
    Create a 3x3 rotation matrix about the y axis
    """
    ry = np.array([[cos(beta),   0, sin(beta)],
                   [0,           1, 0        ],
                   [-sin(beta),  0, cos(beta)]])

    return ry

def rotz(gamma):
    """
    Create a 3x3 rotation matrix about the z axis
    """
    rz = np.array([[cos(gamma), -sin(gamma), 0],
                   [sin(gamma), cos(gamma),  0],
                   [0,          0,           1]])

    return rz

def rotxyz(alpha, beta, gamma):
    """
    Create a 3x3 rotation matrix about the x,y,z axes
    """
    return rotx(alpha).dot(roty(beta)).dot(rotz(gamma))


def homog_transxyz(dx, dy, dz):
    """
    Create a 4x4 homogeneous translation matrix (translation along the x,y,z axes)
    """
    trans = np.array([[1, 0, 0, dx],
                      [0, 1, 0, dy],
                      [0, 0, 1, dz],
                      [0, 0, 0, 1 ]])
    return trans


def homog_transform(dx,dy,dz,alpha,beta,gamma):
    """
    Create a homogeneous 4x4 transformation matrix
    """
    rot4x4 = np.eye(4)
    rot4x4[:3,:3] = rotxyz(alpha,beta,gamma)
    return np.dot(homog_transxyz(dx,dy,dz),rot4x4)


def homog_transform_inverse(matrix):
    """
    Return the inverse of a homogeneous transformation matrix.
                 ------------------------- 
                 |           |           |  
    inverse   =  |    R^T    |  -R^T * d | 
                 |___________|___________| 
                 | 0   0   0 |     1     | 
                 -------------------------  
    """
    inverse = matrix
    inverse[:3,:3] = inverse[:3,:3].T # R^T
    inverse[:3,3] = -np.dot(inverse[:3,:3],inverse[:3,3]) # -R^T * d
    return inverse

def euler_from_quaternion(x,y,z,w):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quaternion = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

def quaternion_from_euler(roll, pitch, yaw):
    """
    Converts euler roll, pitch, yaw to quaternion (w in last place)
    quat = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = [0] * 4
    q[0] = cy * cp * cr + sy * sp * sr
    q[1] = cy * cp * sr - sy * sp * cr
    q[2] = sy * cp * sr + cy * sp * cr
    q[3] = sy * cp * cr - cy * sp * sr

    return q