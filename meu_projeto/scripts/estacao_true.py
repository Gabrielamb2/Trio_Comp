

def printa_resultado(procura_base, objeto, capturou, x_medio_t):
    x_medio = 0
    #print(r)
    if capturou == False:
        procura_base = False

    if procura_base == True and capturou == True:
        bic = True
        x_medio = x_medio_t
        return bic, x_medio, procura_base

    else:
    
        return False, x_medio, procura_base
        
    return False, 0, procura_base
    
            
        

