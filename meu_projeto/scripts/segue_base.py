# This Python file uses the following encoding: utf-8
import os, sys
from geometry_msgs.msg import Twist, Vector3, Pose, Vector3Stamped
import rospy
import garra_demo

def encontrou_base_certa(capturou, bic, base_encontrada, x_medio,medida, centro,velocidade_saida, missao_concluida):

    if capturou == True and bic == True:   
        
        print("ESTADO: BASE ENCONTRADA")

        base_encontrada = True

        velocidade = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
        velocidade_saida.publish(velocidade)
        rospy.sleep(0.1)

        diferenca_base = abs(centro[0] - x_medio)

        if x_medio > centro[0]: #CONDIÇÃO DE DESALINHAMENTO
            velocidade = Twist(Vector3(0, 0, 0), Vector3(0, 0, -0.1))
            velocidade_saida.publish(velocidade)
            rospy.sleep(0.1)

        elif x_medio < centro[0]: #CONDIÇÃO DE DESALINHAMENTO
            velocidade = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0.1))
            velocidade_saida.publish(velocidade)
            rospy.sleep(0.1)

        if diferenca_base <= 40: #ALINHADO!
            if medida < 0.7:
                velocidade = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
                velocidade_saida.publish(velocidade)
                rospy.sleep(2)

                objeto_garra = garra_demo.MoveGroupPythonIntefaceTutorial()

                objeto_garra.inicial()
                objeto_garra.pega_creeper()
                objeto_garra.open_gripper()
                
                print("ESTADO: MISSÃO COMPLETA")
                missao_concluida = True
                
            else:    
                velocidade = Twist(Vector3(0.1, 0, 0), Vector3(0, 0, 0))
                velocidade_saida.publish(velocidade)
                rospy.sleep(2)

    return base_encontrada, missao_concluida