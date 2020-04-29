# Projeto 1




# Rubrica


C - Ficar na pista, encontrar creeper da cor certa, derrubar e voltar para a pista

B - Pegar um creeper da cor certa, voltar para a pista e deixar na base 

A - Pegar creeper da cor e Id certos. Deixar na base certa. Estar bem modularizado 


# Como rodar 

Atualizar `my_simulation` 

Atualizar `robot20`  


# Rodar script instala_garra.sh:

O comando abaixo é capaz de executar o comando:

    sudo sh $(rospack find my_simulation)/garra/instala_garra.sh
    

# Rodar cenário:

    roslaunch my_simulation proj1.launch
 


# Rodar exemplo 

    rosrun projeto1_base base_proj.py
    
# Usando a garra

Iniciando o módulo de controle da garra:

    roslaunch turtlebot3_manipulation_moveit_config move_group.launch

Rodando o software para controlar a garra:

    rosrun my_simulation  open_manipulator.py


