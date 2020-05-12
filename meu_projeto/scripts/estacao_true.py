def printa_resultado(resultados, objeto, capturou):

    for r in resultados:
        if r[0] == objeto and capturou == True:
            bic = True
            ponto_x1 = int(r[2][0])
            ponto_x2 = int(r[3][0])
            x_medio = int((ponto_x1+ponto_x2)/2)

            return bic, x_medio

        else:

            return False, int(0) 
    
            
        

