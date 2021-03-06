#! /usr/bin/env python
# -*- coding:utf-8 -*-

from __future__ import print_function, division
import rospy
import numpy as np
import os, sys
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
import garra_demo

#FUNÇÕES
import estacao_true
import area_creeper
import segue_faixa_amarela
import identifica_creeper
import segue_base

garra = garra_demo.MoveGroupPythonIntefaceTutorial()
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
base_encontrada = False
bic = False
creeper = False
id_certo = False
color = False
orientacao = None
procura_base = False
x_medio_t = 0
missao_concluida = False

missao = ["green", 21, "dog"] 

cor_creeper = missao[0]
id_creeper = missao[1]
objeto = missao[2]

area = 0.0 # Variavel com a area do maior contorno

# Descarta imagens que chegam atrasadas demais
check_delay = False 

resultados = [] # Criacao de uma variavel global para guardar os resultados vistos

x = 0
y = 0
z = 0
id = 0

frame = "camera_link"

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

        #print(tf_buffer.can_transform(frame, marcador, rospy.Time(0)))
        header = Header(frame_id=marcador)
        # Procura a transformacao em sistema de coordenadas 
        trans = tf_buffer.lookup_transform(frame, marcador, rospy.Time(0))
        
        # Separa as translacoes das rotacoes
        x = trans.transform.translation.x
        y = trans.transform.translation.y
        z = trans.transform.translation.z


# A função a seguir é chamada sempre que chega um novo frame
def roda_todo_frame(imagem):

    global cv_image
    global media
    global centro
    global resultados
    global temp_image
    global maior_area
    global procura_base 
    global x_medio_t

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
        centro, temp_image, resultados =  visao_module.processa(temp_image)
        media, maior_area =  cormodule.identifica_cor(temp_image, cor_creeper)        
        for r in resultados:
            if objeto == r[0]:
                procura_base = True
                ponto_x1 = int(r[2][0])
                ponto_x2 = int(r[3][0])
                x_medio_t = int((ponto_x1+ponto_x2)/2)
                     
            

        depois = time.clock()
    except CvBridgeError as e:
        print('ex', e)
    
if __name__=="__main__":

    rospy.init_node("cor")

    topico_imagem = "/camera/rgb/image_raw/compressed"

    recebedor1 = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=4, buff_size = 2**24)
    recebedor2 = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, recebe) # Para recebermos notificacoes de que marcadores foram vistos
    recebedor3 = rospy.Subscriber("/scan", LaserScan, scaneou)

    velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

    tfl = tf2_ros.TransformListener(tf_buffer) #conversao do sistema de coordenadas 
    tolerancia = 25


    try:
        garra.inicial()
        while not rospy.is_shutdown():          

            if temp_image is not None:
                cx = ponto.image_callback(temp_image)[0]

            
                # Note que o imshow precisa ficar *ou* no codigo de tratamento de eventos *ou* no thread principal, não em ambos
                cv2.imshow("cv_image no loop principal", temp_image)
                cv2.waitKey(1)

                # IDENTIFICA A BASE COM O OBJETO DESEJADO--------------------------------------------
                bic, x_medio, procura_base = estacao_true.printa_resultado(procura_base,objeto,capturou,x_medio_t)
                
                # VERIFICA A ÁREA DO CREEPER---------------------------------------------------------
                color = area_creeper.calcula_area(maior_area,capturou)
               
                #PROCURA A FAIXA AMARELA-------------------------------------------------------------
                orientacao = segue_faixa_amarela.procura_faixa_amarela(base_encontrada, creeper, cx, centro, velocidade_saida, orientacao)
               
                #IDENTIFICA O CREEPER DE COR CERTA E ID CERTOS---------------------------------------
                capturou, color, creeper, id_certo  = identifica_creeper.encontra_creeper(id, creeper, base_encontrada, capturou, id_certo, color, id_creeper, x, y, centro, medida, media, velocidade_saida)
                
                #IDENTIFICA A BASE CERTA PARA DEPOSITAR O CREEPER-------------------------------------
                base_encontrada, missao_concluida = segue_base.encontrou_base_certa(capturou, bic, base_encontrada, x_medio, medida,centro, velocidade_saida, missao_concluida)
                

                if missao_concluida == True:
                    break

            rospy.sleep(0.1)

    except rospy.ROSInterruptException:
        print("Ocorreu uma exceção com o rospy")
