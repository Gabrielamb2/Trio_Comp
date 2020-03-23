#! /usr/bin/env python
# -*- coding:utf-8 -*-

#Alunos: Fernando Fincatti, Ellen Shen e Gabriela Boriero

import rospy
from geometry_msgs.msg import Twist, Vector3

v = 0.18  # Velocidade linear
v_2 = 0.09
w = 0.27  # Velocidade angular

if __name__ == "__main__":
    rospy.init_node("roda_exemplo")
    pub = rospy.Publisher("cmd_vel", Twist, queue_size=3)

    try:
        while not rospy.is_shutdown():
	    vel_2 = Twist(Vector3(0,0,0), Vector3(0,0,w))
	    pub.publish(vel_2)
	    rospy.sleep(6)
	    vel_1 = Twist(Vector3(0,0,0), Vector3(0,0,0))
	    pub.publish(vel_1)
	    rospy.sleep(2.0)
	    vel_4 = Twist(Vector3(v_2,0,0), Vector3(0,0,0))
            pub.publish(vel_4)
	    rospy.sleep(4.0)
            vel = Twist(Vector3(v,0,0), Vector3(0,0,0))
            pub.publish(vel)
	    rospy.sleep(3.0)
	    vel_3 = Twist(Vector3(v_2,0,0), Vector3(0,0,0))
            pub.publish(vel_3)
	    rospy.sleep(4.0)
	    
	    

    except rospy.ROSInterruptException:
        print("Ocorreu uma exceção com o rospy")
