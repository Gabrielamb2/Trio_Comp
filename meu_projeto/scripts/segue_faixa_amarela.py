# This Python file uses the following encoding: utf-8
import os, sys
from geometry_msgs.msg import Twist, Vector3, Pose, Vector3Stamped
import rospy

def procura_faixa_amarela(base_encontrada, creeper, cx, centro, velocidade_saida):
    orientacao = 0
    print("ESTADO: SEGUINDO FAIXA AMARELA")

    if cx is not None: #CONDIÇÃO CASO O ROBÔ ENCONTRE A FAIXA AMARELA
        diferenca = abs(centro[0] - cx)

        if cx > centro[0]: #CONDIÇÃO DE DESALINHAMENTO
            #print("DIREITA!")
            orientacao = 1
            velocidade = Twist(Vector3(0, 0, 0), Vector3(0, 0, -0.1))
            velocidade_saida.publish(velocidade)
            rospy.sleep(0.25)
        
        elif cx < centro[0]: #CONDIÇÃO DE DESALINHAMENTO
            #print("ESQUERDA!")
            orientacao = -1
            velocidade = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0.1))
            velocidade_saida.publish(velocidade)
            rospy.sleep(0.25)
        
        if diferenca <= 50: #ALINHADO!
            #print("FRENTE!")
            velocidade = Twist(Vector3(0.1, 0, 0), Vector3(0, 0, 0))
            velocidade_saida.publish(velocidade)
            rospy.sleep(0.5)
        
    else: #CONDIÇÃO CASO O ROBÔ NÃO ENCONTRE A FAIXA AMARELA

        print("ESTADO: PROCURANDO FAIXA AMARELA")

        if orientacao == 1:

            velocidade = Twist(Vector3(0, 0, 0), Vector3(0, 0, -0.1))
            velocidade_saida.publish(velocidade)
            rospy.sleep(0.2)

            velocidade = Twist(Vector3(0.07, 0, 0), Vector3(0, 0, 0))
            velocidade_saida.publish(velocidade)
            rospy.sleep(0.1)

            velocidade = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0.05))
            velocidade_saida.publish(velocidade)
            rospy.sleep(0.1)

        elif orientacao == -1:

            velocidade = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0.1))
            velocidade_saida.publish(velocidade)
            rospy.sleep(0.2)

            velocidade = Twist(Vector3(0.07, 0, 0), Vector3(0, 0, 0))
            velocidade_saida.publish(velocidade)
            rospy.sleep(0.1)

            velocidade = Twist(Vector3(0, 0, 0), Vector3(0, 0, -0.05))
            velocidade_saida.publish(velocidade)
            rospy.sleep(0.1)

    return 



