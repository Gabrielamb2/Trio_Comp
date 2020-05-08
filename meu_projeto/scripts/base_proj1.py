#! /usr/bin/env python
# -*- coding:utf-8 -*-

from __future__ import print_function, division
import rospy
import numpy as np
import numpy
import tf
import math
import cv2
import time
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from numpy import linalg
from tf import transformations
from tf import TransformerROS
import tf2_ros
from geometry_msgs.msg import Twist, Vector3, Pose, Vector3Stamped
from ar_track_alvar_msgs.msg import AlvarMarker, AlvarMarkers
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan

import visao_module
import linha1
import cormodule
import codigo_extras


bridge = CvBridge()
cv_image = None
media = []
centro = []
atraso = 1.5E9 # 1 segundo e meio. Em nanossegundos
ponto = linha1.Follower()
temp_image = None
maior_area = None
medida = None
capturou = False
base_alinhada = False
bic = False

missao = ['blue', 13, 'bicycle'] 

cor_creeper = missao[0]
id_creeper = missao[1]
objeto = missao[2]



area = 0.0 # Variavel com a area do maior contorno

# Só usar se os relógios ROS da Raspberry e do Linux desktop estiverem sincronizados. 
# Descarta imagens que chegam atrasadas demais
check_delay = False 

resultados = [] # Criacao de uma variavel global para guardar os resultados vistos

x = 0
y = 0
z = 0 
id = 0

frame = "camera_link"
# frame = "head_camera"  # DESCOMENTE para usar com webcam USB via roslaunch tag_tracking usbcam

tfl = 0

tf_buffer = tf2_ros.Buffer()

def scaneou(dado):
    global medida
    medida = np.array(dado.ranges).round(decimals=2)[0]
    

def recebe(msg):
    global x # O global impede a recriacao de uma variavel local, para podermos usar o x global ja'  declarado
    global y
    global z
    global id
    for marker in msg.markers:
        id = marker.id
        marcador = "ar_marker_" + str(id)

        print(tf_buffer.can_transform(frame, marcador, rospy.Time(0)))
        header = Header(frame_id=marcador)
        # Procura a transformacao em sistema de coordenadas entre a base do robo e o marcador numero 100
        # Note que para seu projeto 1 voce nao vai precisar de nada que tem abaixo, a 
        # Nao ser que queira levar angulos em conta
        trans = tf_buffer.lookup_transform(frame, marcador, rospy.Time(0))
        
        # Separa as translacoes das rotacoes
        x = trans.transform.translation.x
        y = trans.transform.translation.y
        z = trans.transform.translation.z
        # ATENCAO: tudo o que vem a seguir e'  so para calcular um angulo
        # Para medirmos o angulo entre marcador e robo vamos projetar o eixo Z do marcador (perpendicular) 
        # no eixo X do robo (que e'  a direcao para a frente)
        t = transformations.translation_matrix([x, y, z])
        # Encontra as rotacoes e cria uma matriz de rotacao a partir dos quaternions
        r = transformations.quaternion_matrix([trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w])
        m = numpy.dot(r,t) # Criamos a matriz composta por translacoes e rotacoes
        z_marker = [0,0,1,0] # Sao 4 coordenadas porque e'  um vetor em coordenadas homogeneas
        v2 = numpy.dot(m, z_marker)
        v2_n = v2[0:-1] # Descartamos a ultima posicao
        n2 = v2_n/linalg.norm(v2_n) # Normalizamos o vetor
        x_robo = [1,0,0]
        cosa = numpy.dot(n2, x_robo) # Projecao do vetor normal ao marcador no x do robo
        angulo_marcador_robo = math.degrees(math.acos(cosa))

        # Terminamos
        print("id: {} x {} y {} z {} angulo {} ".format(id, x,y,z, angulo_marcador_robo))



# A função a seguir é chamada sempre que chega um novo frame
def roda_todo_frame(imagem):
    #print("frame")
    global cv_image
    global media
    global centro
    global resultados
    global temp_image
    global maior_area

    now = rospy.get_rostime()
    imgtime = imagem.header.stamp
    lag = now-imgtime # calcula o lag
    delay = lag.nsecs
    if delay > atraso and check_delay==True:
        print("Descartando por causa do delay do frame:", delay)
        return 
    try:
        antes = time.clock()
        temp_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
        # Note que os resultados já são guardados automaticamente na variável
        # chamada resultados
        centro, saida_net, resultados =  visao_module.processa(temp_image)
        media, maior_area =  cormodule.identifica_cor(temp_image, cor_creeper)        
        for r in resultados:
            # print(r) - print feito para documentar e entender
            # o resultado            
            pass

        depois = time.clock()
        # Desnecessário - Hough e MobileNet já abrem janelas
        cv_image = saida_net.copy()
    except CvBridgeError as e:
        print('ex', e)
    
if __name__=="__main__":
    rospy.init_node("cor")

    topico_imagem = "/camera/rgb/image_raw/compressed"

    recebedor1 = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=4, buff_size = 2**24)
    recebedor2 = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, recebe) # Para recebermos notificacoes de que marcadores foram vistos
    recebedor3 = rospy.Subscriber("/scan", LaserScan, scaneou)

    #print("Usando ", topico_imagem)

    velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

    tfl = tf2_ros.TransformListener(tf_buffer) #conversao do sistema de coordenadas 
    tolerancia = 25

    # Exemplo de categoria de resultados
    # [('chair', 86.965459585189819, (90, 141), (177, 265))]

    try:
        # Inicializando - por default gira no sentido anti-horário
        # vel = Twist(Vector3(0,0,0), Vector3(0,0,math.pi/10.0))
        
        while not rospy.is_shutdown():



        # IDENTIFICA A BASE COM O OBJETO DESEJADO-------------------------------------

            for r in resultados:
                print("RESULTADO: ",r[0])
                if r[0] == objeto:
                    bic = True
                    ponto_x1 = int(r[2][0])
                    ponto_x2 = int(r[3][0])
                    x_medio = (ponto_x1+ponto_x2)/2

        # IDENTIFICA A BASE COM O OBJETO DESEJADO-------------------------------------            
            





            if cv_image is not None and temp_image is not None:
                cx = ponto.image_callback(temp_image)[0]



                
                # Note que o imshow precisa ficar *ou* no codigo de tratamento de eventos *ou* no thread principal, não em ambos
                cv2.imshow("cv_image no loop principal", temp_image)
                cv2.waitKey(1)

                




                #IDENTIDICA SE O CREEPER FOI ENCONTRADO OU NÃO-------------------------------------

                if maior_area >= 250 and capturou == False: #CONDIÇÃO PARA DETERMINAR SE O CREEPER DA COR DESEJADA FOI ECONTRADO
                    color = True

                elif maior_area <= 250 and capturou == False: #CONDIÇÃO PARA DETERMINAR SE O CREEPER DA COR DESEJADA FOI ECONTRADO
                    color = False

                #IDENTIDICA SE O CREEPER FOI ENCONTRADO OU NÃO------------------------------------
            
                


                #PROCURA A FAIXA AMARELA-------------------------------------
                if base_alinhada == False:
                    if color == False and bic == False or id != id_creeper: #SE NÃO ECONTROU O CREEPER AINDA...
                        #print("COLOR É FALSE")
                        if cx is not None: #CONDIÇÃO CASO O ROBÔ ENCONTRE A FAIXA AMARELA
                            diferenca = abs(centro[0] - cx)

                            if cx > centro[0]: #CONDIÇÃO DE DESALINHAMENTO
                                #print("DIREITA!")
                                velocidade = Twist(Vector3(0, 0, 0), Vector3(0, 0, -0.1))
                                velocidade_saida.publish(velocidade)
                                rospy.sleep(0.1)
                            
                            elif cx < centro[0]: #CONDIÇÃO DE DESALINHAMENTO
                                #print("ESQUERDA!")
                                velocidade = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0.1))
                                velocidade_saida.publish(velocidade)
                                rospy.sleep(0.1)
                            
                            if diferenca <= 15: #ALINHADO!
                                #print("FRENTE!")
                                velocidade = Twist(Vector3(0.1, 0, 0), Vector3(0, 0, 0))
                                velocidade_saida.publish(velocidade)
                                rospy.sleep(0.25)
                            
                        else: #CONDIÇÃO CASO O ROBÔ NÃO ENCONTRE A FAIXA AMARELA
                            print("PROCURANDO FAIXA AMARELA")
                            velocidade = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0.05))
                            velocidade_saida.publish(velocidade)
                            rospy.sleep(0.1)

                #PROCURA A FAIXA AMARELA-------------------------------------





                #IDENTIFICA O CREEPER DE COR CERTA E ID CERTOS-------------------------------------

                elif color == True and capturou == False and id == id_creeper: #CREEPER ENCONTRADO!
                    print("COLOR É TRUE")
                    diferenca_cor = abs(centro[0]-y)

                    if x > 0.85 and id == id_creeper: #CASO O CREEPER ESTEJA LONGE, ANDE ATÉ ELE
                        id_certo = False
                        if y < -0.05: #CONDIÇÃO DE DESALINHAMENTO
                            vel = Twist(Vector3(0,0,0), Vector3(0,0,-0.05))
                            velocidade_saida.publish(vel)
                            rospy.sleep(0.1)
 
                        elif y > 0.05: #CONDIÇÃO DE DESALINHAMENTO
                            vel = Twist(Vector3(0,0,0), Vector3(0,0,0.05))
                            velocidade_saida.publish(vel)
                            rospy.sleep(0.1)

                        if diferenca_cor <= 320.05 and diferenca_cor >= 319.95: #ALINHADO!
                            print("ALINHOU COM O CREEPER!")
                            velocidade = Twist(Vector3(0.07, 0, 0), Vector3(0, 0, 0))
                            velocidade_saida.publish(velocidade)
                            rospy.sleep(0.1)

                    else:
                        id_certo = True
                            
                    if id_certo == True: #O CREEPER ESTÁ PERTO, APROXIMAR LENTAMENTE
                        if media[0] > centro[0]: #CONDIÇÃO DE DESALINHAMENTO
                            vel = Twist(Vector3(0,0,0), Vector3(0,0,-0.05))
                            velocidade_saida.publish(vel)
                            rospy.sleep(0.1)
 
                        elif media[0] < centro[0]: #CONDIÇÃO DE DESALINHAMENTO
                            vel = Twist(Vector3(0,0,0), Vector3(0,0,0.05))
                            velocidade_saida.publish(vel)
                            rospy.sleep(0.1)

                        if abs(media[0] - centro[0]) < 5: #ALINHADO!
                            velocidade = Twist(Vector3(0.03, 0, 0), Vector3(0, 0, 0))
                            velocidade_saida.publish(velocidade)
                            rospy.sleep(0.1)


                        if  medida <= 0.25: #CREEPER PRONTO PARA SER CAPTURADO!
                            print("PREPARAR A GARRA")
                            capturou = True
                            velocidade = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
                            velocidade_saida.publish(velocidade)
                            rospy.sleep(0.1)

                            velocidade = Twist(Vector3(-0.1, 0, 0), Vector3(0, 0, 0))
                            velocidade_saida.publish(velocidade)
                            rospy.sleep(8)
                            color = False

                #IDENTIFICA O CREEPER DE COR CERTA E ID CERTOS-------------------------------------





                #IDENTIFICA A BASE CERTA PARA DEPOSITAR O CREEPER-------------------------------------

                if capturou == True and bic == True:

                    diferenca_base = centro[0] - x_medio

                    if base_alinhada == False:
                        if x_medio > centro[0]: #CONDIÇÃO DE DESALINHAMENTO
                            velocidade = Twist(Vector3(0, 0, 0), Vector3(0, 0, -0.1))
                            velocidade_saida.publish(velocidade)
                            rospy.sleep(0.1)

                        elif x_medio < centro[0]: #CONDIÇÃO DE DESALINHAMENTO
                            velocidade = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0.1))
                            velocidade_saida.publish(velocidade)
                            rospy.sleep(0.1)

                        if diferenca_base <= 5: #ALINHADO!
                            base_alinhada = True

                            cv2.circle(image, (x_medio, centro[1]), 20, (0,0,255), -1)

                            velocidade = Twist(Vector3(0.1, 0, 0), Vector3(0, 0, 0.1))
                            velocidade_saida.publish(velocidade)
                            rospy.sleep(2)

                            if x_medio > centro[0]: #CONDIÇÃO DE DESALINHAMENTO
                                velocidade = Twist(Vector3(0, 0, 0), Vector3(0, 0, -0.1))
                                velocidade_saida.publish(velocidade)
                                rospy.sleep(0.1)

                            elif x_medio < centro[0]: #CONDIÇÃO DE DESALINHAMENTO
                                velocidade = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0.1))
                                velocidade_saida.publish(velocidade)
                                rospy.sleep(0.1)

                            print("Base alinhada")

                    else:
                        pass

            #IDENTIFICA A BASE CERTA PARA DEPOSITAR O CREEPER-------------------------------------
                print("BICICLETA: ", bic)
                print("COR: ", color)
                print("id: ", id)
                print("base_alinhada: ", base_alinhada)
                print("capturou: ", capturou)
                print("id_certo: ", id_certo)



            rospy.sleep(0.1)

    except rospy.ROSInterruptException:
        print("Ocorreu uma exceção com o rospy")
