#### Instituto Infnet
#### Disicplina: Veículos Autônomos
#### Prof.: M.e. Adalberto Oliveira
#### Assessment

Qst1- Durante a disciplina foram trabalhadas as etapas que permitem que um veículo robótico realize tarefas de maneira autônoma: controle de posição/postura, captura e tratamento de imagens, extração de dados a partir de uma imagem e controle de robôs utilizando os atributos da imagem, podendo ser do tipo IBVS (Image Based Visual Servoing) ou PVBS (Position Based Visual Servoing). Com base nestes conceitos e utilizando todo o material disponibilizado, desenvolva um pacote para o framework ROS que seja capaz de controlar o simulador TurtleBot3, modelo Waffle (Figura 1) para que ele realize uma tarefa de regulação de posição utilizando como base a abordagem de controle visual PVBS, encontrando a posição no mundo do alvo desejado, e se deslocando até ele de maneira autônoma.

O trabalho deverá atender aos seguintes requisitos:

- Encontrar um objeto posicionado dentro do simulador, desde que dentro do limite de visão da câmera;
- Segmentar os objetos para extrair os seus atributos;
- Extrair o atributo “Ponto da Base” e sua coordenada em pixel;
- Encontrar a distância entre este ponto e o robô;
- Transformar o ponto do referencial da imagem para o referencial do mundo e utilizar como coordenada de destino;
- Controlar o robô de forma que ele chegue em um ponto a 30cm de distância da coordenada da base do objeto (regulação parcial);
- Exibir as telas de imagem original, máscara de segmentação e imagem processada;
- Informar a distância entre o robô e o objeto de destino, coordenada em pixel do ponto da base e a coordenada do objeto no mundo, ou por impressão no terminal ou por publicação no ROS (info ou publicação de mensagem);
- A aplicação deverá ser executada a partir de um arquivo *.launch, que deverá iniciar todos os nós do projeto, caso seja utilizado mais de um nó, assumindo que o simulador já esteja em execução. Os seguintes elementos deverão ser entregues como parte da avaliação:


#### MANUAL TÉCNICO + EVIDÊNCIAS 

#### % Etapa 1: Criando o pacote ec_at_va
```shell
$ initros1
$ source ~/catkin_ws/devel/setup.bash
$ cd ~/catkin_ws/src/
$ catkin_create_pkg ec_at_va rospy geometry_msgs
```

#### % Etapa 2: Realizada a adaptação dos scripts/launchs cedidos pelo professor para:

#### 2.1: calibrador_hsv_yaml_ROS.py
- **Função** = Criação da máscara para binarização do objeto lido no tópico informado via input, salvando os parâmetros (limites superiores e inferiores escolhidos) no arquivo *‘cfg/mask_param.yaml’*.

#### 2.2: get_point_from_message.launch
- **Nó** = get_point_message
- **Função** = Coletada através da leitura de um tópico da câmera do Turtlebot3 no Gazebo, realizar o processamento da imagem utilizando os parâmetros da máscara e aplicando filtros para de forma a realizar a segmentação do objeto na cena, envolvendo-o com um boundbox, calculando o ponto central da base exibindo-o e publicando essa informação em um tópico.
- **Script** = *get_point_from_message.py*
- **Parâmetros de entrada** = realiza a leitura dos limites superior e inferior da máscara contidos no arquivo *‘mask_param.yaml’* e se a janela de visualização do resultado da aplicação do processo de máscara na imagem lida deverá ou não ser apresentada (show_image).
- **Tópicos** = realiza leitura do tópico *‘/camera/rgb/image_raw’* e escrita no tópico *‘/camera/img_base’*.

#### 2.3: tb3_transformations.launch
- **Nó** = tb3_tf_broadcast
- **Função** = Realizar a árvore de transformações necessária para que o ponto em pixel ao final seja uma coordenada do objeto no mundo.
- **Tópico** = realiza leitura do tópico *‘/odom’*.


#### % Etapa 3: Criação do launch ‘tb3_visual_servoing_pbvs.launch’, responsável por chamar os outros launchers, iniciando assim todos os nós necessários de uma única vez, bem como carregar os parâmetros/script necessários realização do PBVS.
- **Nó** = visual_servoing
- **Função** = Controlar a movimentação do Turtlebot3 por meio de postura parcial, tendo em vista que o mesmo ficará a uma distância de 30 cm do objeto cuja ponto da base na imagem em pixel está sendo publicado pelo nó get_point_message e será convertido para uma coordenada da câmera através da função get_img_point que para realizar essa tarefa recebe as informações da câmera a fim de realizar os cálculos de transformação necessários. Depois será convertido para uma coordenada no mundo pelo tópico tb3_tf_broadcast, onde finalmente será o ponto de destino a ser alcançado pelo robô através da função  cartesian_control que realiza os cálculos do sinal de controle para orientar o robô e é baseado no algoritmo de controle cartesiano, utilizando a coordenada em que se encontra o robô e a de destino para verificar a diferença que falta ser alcançada aliado ao PID (controle de ganhos) para ajustar a postura de forma suave. O processo de controle do robô inicia-se automaticamente a partir do momento em que é lida a informação do ponto da base da imagem, ou seja, que o objeto foi detectado, do contrário, o robô se manterá em rotação com velocidade angular de 0.5 para realizar a busca do objeto limitado a seu campo de visão. O resultado final é um controle realizado no campo de visão do robô que estima a posição do objeto em relação à câmera (PBVS = Position-Based Visual Servo):
- **Script** = visual_servoing_pbvs.py
- **Parâmetros** = ganhos do controlador PID para regulação de postura (K_eu e K_ev, velocidade linear e velocidade angular respectivamente, ambos com 0.25, sendo assim o robô irá realizar seu deslocamento e ajuste de orientação em uma velocidade considerada adequada), altura da câmera em relação ao robô (10cm do solo)
Tópicos = realizada a leitura dos tópicos ‘/camera/img_base’, ‘/camera/rgb/camera_info’ e ‘/odom’, e publica no tópico  ‘/cmd_vel’


#### % Etapa 4: Dar permissão de execução (chmod +x) para os scripts abaixo:
- *calibrador_hsv_yaml_ROS.py*
- *get_point_from_message.py*
- *tb3_tf_broadcast.py*
- *visual_servoing_pbvs.py*


#### % Etapa 5: Build do pacote
```shell
$ cd ~/catkin_ws/
$ catkin_make
```


#### % Etapa 6: Processamento da imagem e criação da máscara do objeto. O resultado contendo os valores dos limites superiores e inferiores e salvo no arquivo ‘mask_param.yaml’ informado via input, bem como o número de máscaras criadas.
```shell
$ cd ~/catkin_ws/src/ec_at_va/cfg
$ rosrun ec_at_va calibrador_hsv_yaml_ROS.py
```



#### % Etapa 7: Inicializando o Gazebo com o cenário ‘empty_world’ do TurtleBot3 e inserir o objeto (modelo) ‘Wood cube 10cm’, posicionando o mesmo a uma dist ncia considerável mas sem sair do raio de visão da câmera 
```shell
$ initros1
$ source catkin_ws/devel/setup.bash
$ export TURTLEBOT3_MODEL=waffle_pi
$ roslaunch turtlebot3_gazebo turtlebot3_empty_world.lanch
```


#### % Etapa 8: Inicializar o launch para regulação de postura parcial (30cm de distância do objeto - para isso foi configurado um threshold de 0.37) utilizando a técnica de PBVS
```shell
$ initros1
$ source catkin_ws/devel/setup.bash
$ roslaunch ec_at_va tb3_visual_servoing_pbvs.launch
```

Vídeo de demonstração: https://youtu.be/ZIRpPIw4BBg


<p>------------------------------------</p>

Qst2- O Aprendizado por Reforço é uma técnica de aprendizado de máquina que permite que um agente aprenda a melhor ação a ser tomada em um determinado estado a partir da interação com o ambiente em que está inserido. O objetivo do agente será acumular a maior recompensa possível durante cada época de treinamento. Uma das abordagens de aprendizado é o Deep Q-Learning (DQN), em que uma rede neural para estimar qual a ação que trará a maior recompensa futura durante o treinamento, dado um determinado estado. Com base nesses conceitos e utilizando o material disponibilizado em aula, realize o treinamento de um agente que seja capaz de pousar um drone em uma determinada posição do ambiente, como mostrado na Figura 2. Para isso, a partir dos valores inicialmente configurados, realize o ajuste dos seguintes hiperparâmetros responsáveis pelo processo de aprendizado:

Fator de desconto (gamma);
Fator de exploração (epsilon);
Épocas de treinamento (Episodes);
Tamanho da rede (neurônios por camada);

Escolha pelo menos um desses parâmetros e faça o seu ajuste para avaliar o impacto de sua variação no processo de aprendizado. Para avaliar o processo de aprendizado, construa um gráfico com as recompensas recebidas durante o treinamento em cada configuração utilizada, comparando a evolução da aprendizagem do agente em cada configuração. 

Após o treinamento, avalie o comportamento do agente com cada uma das configurações treinadas, verificando a posição final que o drone atingiu. Para isso, faça cinco rodas de pousos com cada agente e colete a posição final do drone. Para cada agente monte uma tabela com as coordenadas finais em cada pouso, o erro (coordenada desejada – coordenada do pouso) para cada coordenada (x e y), e a média do erro para cada coordenada em cada pouso

Utilizando os dados coletados, faça uma avaliação de qual configuração obteve o melhor resultado, tanto durante o treinamento como durante a inferência, apontando qual é a melhor configuração. 

Extra: realize a variação de mais de um parâmetro e avalie a interação desses parâmetros e seu impacto para o processo de aprendizado e no processo de inferência.
---
