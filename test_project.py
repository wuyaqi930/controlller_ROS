#!/usr/bin/env python
#coding=utf-8

# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# ---------------------------导入相关的package--------------------------- 
import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty
from numpy import * 
import time

# ---------------------------定义turtlebot的速度参数---------------------------
BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 1.82

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1

# ---------------------------定义和e有关的参数---------------------------
e = """
Communications Failed
"""

# ---------------------------定义controller相关参数---------------------------

P_t=mat(zeros((3,1)))#定义状态矩阵（X，Y，theta），并初始化

# P_t[0]=10 # 定义状态矩阵初始数值不为零
# P_t[1]=0
# P_t[2]=10

Pd_t=mat(zeros((3,1))) 

P_derivative_t=mat(zeros((3,1))) #定义状态矩阵的导数（X，Y，theta），并初始化
Pd_derivative_t=mat(zeros((3,1)))

U_t=mat(zeros((2,1))) # 定义输入U（线速度，角速度），并初始化
Ud_t=mat(zeros((2,1))) 

A=mat(zeros((3,2))) #定义运动学方程的转换矩阵，并初始化
Ad=mat(zeros((3,2)))

P_delta=mat(zeros((3,1))) #定义理想状态矩阵和实际状态矩阵的差值，并初始化

E_t=mat(zeros((3,1))) #定义Kanayama类型误差变量，并初始化

B=mat(zeros((3,3)))#定义Kanayama类型误差变量方程的转换矩阵，并初始化

Kx=1      #controller的参数（王凯公式（15））
K_theta=1

# ---------------------------调用函数定义---------------------------

# 检查目标线速度是否在turtlebot相应模型理想范围之内
def checkLinearLimitVelocity(vel):
    if turtlebot3_model == "burger":
      vel = constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)
    elif turtlebot3_model == "waffle" or turtlebot3_model == "waffle_pi":
      vel = constrain(vel, -WAFFLE_MAX_LIN_VEL, WAFFLE_MAX_LIN_VEL)
    else:
      vel = constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)

    return vel

# 检查目标角速度是否在turtlebot相应模型理想范围之内
def checkAngularLimitVelocity(vel):
    if turtlebot3_model == "burger":
      vel = constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)
    elif turtlebot3_model == "waffle" or turtlebot3_model == "waffle_pi":
      vel = constrain(vel, -WAFFLE_MAX_ANG_VEL, WAFFLE_MAX_ANG_VEL)
    else:
      vel = constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)

    return vel

# 将线速度和角速度实时数值显示在界面上
def vels(target_linear_vel, target_angular_vel):
    return "currently:\tlinear vel %s\t angular vel %s " % (target_linear_vel,target_angular_vel)

# 控制control_vel匹配linear,实现按照step_size进行
def makeSimpleProfile(output, input, slop):
    if input > output:
        output = min( input, output + slop )
    elif input < output:
        output = max( input, output - slop )
    else:
        output = input

    return output

# limit the input speed that matches specific model
def constrain(input, low, high):
    if input < low:
      input = low
    elif input > high:
      input = high
    else:
      input = input

    return input

# read a key from the keyboard
def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


# ---------------------------开始执行主程序---------------------------

if __name__=="__main__":

    settings = termios.tcgetattr(sys.stdin)

    # 定义node的名称
    rospy.init_node('turtlebot3_teleop')

    # 创建publisher,发布的话题是“cmd_vel”，频率是10Hz
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    turtlebot3_model = rospy.get_param("model", "burger")


    #---------------------------定义相关变量---------------------------

    # 给理想输入 Ud 赋值（走圆形的话，线速度V和角速度w都是常数
    Ud_t[0]= 0.1
    Ud_t[1]= 0.1

    # 理想和实际线速度角速度s
    status = 0
    target_linear_vel   = 0.0
    target_angular_vel  = 0.0
    control_linear_vel  = 0.0
    control_angular_vel = 0.0

    # 计算理想的线速度和角速度
    try:
        #---------------------------给实际输入 U 赋值（根据相关的公式进行输入）---------------------------
	while(1):
            #计算P_delta，求理想轨迹和实际轨迹的误差
            P_delta = Pd_t - P_t

            #计算Kanayama类型误差变量方程的转换矩阵B

            B[0,0]= cos(P_t[2])  #给转换矩阵B进行赋值
            B[0,1]= sin(P_t[2])
            B[1,0]= -sin(P_t[2])
            B[1,1]= cos(P_t[2])
            B[2,2]= 1
	    
            #计算Kanayama类型误差变量
            E_t = B * P_delta

            # 计算输入 (按道理各个项目应当就到此时能够实现）
	    # V（t）= U_t(0),W(t) = U_t(1)	Vd（t）= Ud_t(0),Wd(t) = Ud_t(1)
	    # e_theta = E_t(2) e_x = E_t(0)

            if (E_t[2] == 0):
                U_t[0] = Ud_t[0]*cos(E_t[2]) + Kx * E_t[0]  #王凯公式（15）需要进行一个判断，否则会出现没有解的情况
                U_t[1]= Ud_t[1] + K_theta * E_t[2] + Ud_t[0]*E_t[1]
            else:
                U_t[0]= Ud_t[0]*cos(E_t[2]) + Kx * E_t[0]  #王凯公式（15）需要进行一个判断，否则会出现没有解的情况
                U_t[1]= Ud_t[1] + K_theta * E_t[2] + Ud_t[0]*E_t[1]*sin(E_t[2]) / E_t[2]
	

            #---------------------------给实际输入 U 赋值（根据相关的公式进行输入）---------------------------

            #给target线速度进行赋值
            target_linear_vel = checkLinearLimitVelocity(U_t[0])

            #给target角速度进行赋值
            target_angular_vel = checkAngularLimitVelocity(U_t[1])
	    
            # 定义twist变量
            twist = Twist()

            #给control线速度进行赋值
            control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
            twist.linear.x = control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0

            #给control角速度进行赋值
            control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel
	    
	    # 显示此时的线速度、角速度数值
            print (vels(control_linear_vel,control_angular_vel))  # 有疑问 是否需要添加括号
	    
            #将control线速度角速度作为消息发布出去
            pub.publish(twist)

	    #stop mobile robot
	    # key = getKey()
	    
	    # if (key == 'c'):
		# twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
                # twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0

                # pub.publish(twist)

	    #sleep 1s
	    time.sleep(1)    

    except:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        pub.publish(twist)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)