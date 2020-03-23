# -*- coding: utf-8 -*-
"""
Created on Mon Mar  2 14:27:31 2020

@author: Fernando
"""

#Alunos: Fernando Fincatti, Ellen Shen e Gabriela Boriero
#Retas e ponto de fuga

import cv2
import numpy as np
import math
import itertools


def eq_reta(x1,x2,y1,y2):
    m= (y2-y1)/(x2-x1)
    h = y1 - m * x1
    y = m * x2 - m* x1 + y1
    return [m,h,y]

def intersec(m1, m2, h1, h2):
    Xi= (h2-h1)/(m1-m2)
    Yi = m1*Xi +h1
    return [Xi, Yi]

def def_angulo(x1,x2,y1,y2):
    termoy = abs(y1-y2)
    termox = abs(x1-x2)
    angulo = math.atan2(termoy,termox)
    angulo = math.degrees(angulo)
    return angulo

def fixa_y(lista_de_retas,y_travado):
    for reta in lista_de_retas:
        m = eq_reta(reta[0],reta[2],reta[1],reta[3])[0]
        x = y_travado - reta[1]+m*reta[0]
        reta.append(x)
    return lista_de_retas

        
cap = cv2.VideoCapture("video1.mp4")
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()
    
    if ret == False:
        print("Codigo de retorno FALSO - problema para capturar o frame")

    #hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    cor_menor = np.array([200, 200, 200])
    cor_maior = np.array([255, 255, 255])
    mascara = cv2.inRange(frame, cor_menor, cor_maior)
    
    img_1 = cv2.bitwise_or(frame, frame, mask=mascara)
    img_cinza = cv2.cvtColor(img_1, cv2.COLOR_BGR2GRAY)
    
    #img_cinza = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    
    edges = cv2.Canny(img_cinza,50,100)
    
    comprimento_minimo = 110
    comprimento_maximo = 120
    
    
    ang_vertical = def_angulo(0,0,10,20) 
    lines=[]
    lines = cv2.HoughLinesP(image=edges,rho=1,theta=np.pi/180,threshold=50,
                            minLineLength=comprimento_minimo, maxLineGap=comprimento_maximo)
    
    angulos = []
    if lines is not None:
        for l in lines:
            for x1, y1, x2, y2 in list(l):
                angulo = def_angulo(x1, x2, y1, y2)
                angulos.append(angulo)
                
    # Capturar somente as retas verticais do frame
    retas_verticais = []
    for i in range(len(angulos)-1):
        if abs(angulos[i]-ang_vertical) < 50:
            l = lines[i]
            l0 =np.array(l[0]).tolist()
            x11 = l0[0]
            y11 = l0[1]
            x22 = l0[2]
            y22 = l0[3]
            reta_vertical = [x11,y11,x22,y22]
            retas_verticais.append(reta_vertical)
    
    #Fixar um X para calcular a distância
    retas_verticais = fixa_y(retas_verticais,150)
    distancia = []
    centro_x = int(frame.shape[0])/2
    maior_diferenca = 0
    
    #Lista de retas que serão desenhdas no frame
    finais = []
    reta_top1 = None
    #Encontrando as duas retas para serem desenhadas no frame
    if retas_verticais is not None:
        for reta1,reta2 in itertools.combinations(retas_verticais, 2):
            if abs(reta1[4]-reta2[4]) > 500 and abs(reta1[4]-reta2[4]) > maior_diferenca:
                maior_diferenca = abs(reta1[4]-reta2[4])
                reta_top1 = reta1
                reta_top2 = reta2
        if reta_top1 is not None:
            for reta in retas_verticais:
                if reta == reta_top1 or reta == reta_top2:
                    finais.append(reta)
                    cv2.line(frame, (int(reta[0]), reta[1]), (reta[2], reta[3]), (0, 255, 0), 3)

    #Calculando ponto de fuga
    if len(finais) > 1:
        reta_final1 = finais[0]
        reta_final2 = finais[1]
        
        x1_1 = reta_final1[0]
        x2_1 = reta_final1[2]
        y1_1 = reta_final1[1]
        y2_1 = reta_final1[3]
        
        x1_2 = reta_final2[0]
        x2_2 = reta_final2[2]
        y1_2 = reta_final2[1]
        y2_2 = reta_final2[3]
        
        eq_reta1 = eq_reta(x1_1,x2_1,y1_1,y2_1)
        eq_reta2 = eq_reta(x1_2,x2_2,y1_2,y2_2)
        
        h1 = eq_reta1[1]
        h2 = eq_reta2[1]
        
        m1 = eq_reta1[0]
        m2 = eq_reta2[0]
         
        ponto_fuga = intersec(m1,m2,h1,h2)
            
        # Window name in which image is displayed 
        window_name = 'Image'
           
        # Center coordinates 
        center_coordinates =(int(ponto_fuga[0]), int(ponto_fuga[1]))
          
        # Radius of circle
        raio = 3
           
        # Blue color in BGR 
        color = (255, 0, 0) 
           
        # Line thickness of 2 px 
        thickness = 4
        
        
        cv2.circle(frame, center_coordinates, raio, color, thickness)
        print("ponto_fuga: ",ponto_fuga)
    
        cv2.imshow('color', frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
