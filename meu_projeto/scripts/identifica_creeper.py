# This Python file uses the following encoding: utf-8
import os, sys
from geometry_msgs.msg import Twist, Vector3, Pose, Vector3Stamped
import rospy
import garra_demo

def encontra_creeper(id, creeper, base_encontrada, capturou, id_certo, color, id_creeper, x, y, centro, medida, media, velocidade_saida):

    if base_encontrada == False and capturou == False:

        if id_certo == False:
            
            if color == True and id == id_creeper: #CREEPER ENCONTRADO!
                
                if x >= 1.5:
                    creeper = False

                elif x<= 1.6 and x >= 1.4:
                    vel = Twist(Vector3(0.1,0,0), Vector3(0,0,0))
                    velocidade_saida.publish(vel)
                    rospy.sleep(0.1)


                else:

                    print("ESTADO: CRERPER ENCONTRADO")

                    creeper = True
                    
                    diferenca_cor = abs(centro[0]-y)


                    if x > 1 and id == id_creeper: #CASO O CREEPER ESTEJA LONGE, ANDE ATÉ ELE
                        id_certo = False
                        if y < -0.10: #CONDIÇÃO DE DESALINHAMENTO
                            vel = Twist(Vector3(0,0,0), Vector3(0,0,-0.1))
                            velocidade_saida.publish(vel)
                            rospy.sleep(0.1)

                        elif y > 0.10: #CONDIÇÃO DE DESALINHAMENTO
                            vel = Twist(Vector3(0,0,0), Vector3(0,0,0.1))
                            velocidade_saida.publish(vel)
                            rospy.sleep(0.1)

                        if diferenca_cor <= 320.15 and diferenca_cor >= 319.85: #ALINHADO!
                            #print("ALINHOU COM O CREEPER!")
                            velocidade = Twist(Vector3(0.1, 0, 0), Vector3(0, 0, 0))
                            velocidade_saida.publish(velocidade)
                            rospy.sleep(0.5)

                    else:
                        id_certo = True
                        
        elif id_certo == True: #O CREEPER ESTÁ PERTO, APROXIMAR LENTAMENTE

            if medida > 0.5:

                if media[0] > centro[0]: #CONDIÇÃO DE DESALINHAMENTO
                    vel = Twist(Vector3(0,0,0), Vector3(0,0,-0.1))
                    velocidade_saida.publish(vel)
                    rospy.sleep(0.1)

                elif media[0] < centro[0]: #CONDIÇÃO DE DESALINHAMENTO
                    vel = Twist(Vector3(0,0,0), Vector3(0,0,0.1))
                    velocidade_saida.publish(vel)
                    rospy.sleep(0.1)

                if abs(media[0] - centro[0]) < 10: #ALINHADO!
                    velocidade = Twist(Vector3(0.1, 0, 0), Vector3(0, 0, 0))
                    velocidade_saida.publish(velocidade)
                    rospy.sleep(0.1)

            elif medida <= 0.5:

                if media[0] > centro[0]: #CONDIÇÃO DE DESALINHAMENTO
                    vel = Twist(Vector3(0,0,0), Vector3(0,0,-0.05))
                    velocidade_saida.publish(vel)
                    rospy.sleep(0.1)

                elif media[0] < centro[0]: #CONDIÇÃO DE DESALINHAMENTO
                    vel = Twist(Vector3(0,0,0), Vector3(0,0,0.05))
                    velocidade_saida.publish(vel)
                    rospy.sleep(0.1)

                if abs(media[0] - centro[0]) < 10: #ALINHADO!
                    velocidade = Twist(Vector3(0.03, 0, 0), Vector3(0, 0, 0))
                    velocidade_saida.publish(velocidade)
                    rospy.sleep(0.1)


            if  medida <= 0.25: #CREEPER PRONTO PARA SER CAPTURADO!
                #print("PREPARAR A GARRA")
                capturou = True
                color = False
                creeper = False
                velocidade = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
                velocidade_saida.publish(velocidade)
                rospy.sleep(0.1)

                garra_demo.main()

                velocidade = Twist(Vector3(-0.1, 0, 0), Vector3(0, 0, 0))
                velocidade_saida.publish(velocidade)
                rospy.sleep(8)


    return capturou, color, creeper, id_certo



