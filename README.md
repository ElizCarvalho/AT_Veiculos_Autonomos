# AT_Veiculos_Autonomos

#### Instituto Infnet
#### Disicplina: Veículos Autônomos
#### Prof.: M.e. Adalberto Oliveira
#### Assessment

Qst1- Durante a disciplina foram trabalhadas as etapas que permitem que um veículo robótico realize tarefas de maneira autônoma: controle de posição/postura, captura e tratamento de imagens, extração de dados a partir de uma imagem e controle de robôs utilizando os atributos da imagem, podendo ser do tipo IBVS (Image Based Visual Servoing) ou PVBS (Position Based Visual Servoing). Com base nestes conceitos e utilizando todo o material disponibilizado, desenvolva um pacote para o framework ROS que seja capaz de controlar o simulador TurtleBot3, modelo Waffle (Figura 1) para que ele realize uma tarefa de regulação de posição utilizando como base a abordagem de controle visual PVBS, encontrando a posição no mundo do alvo desejado, e se deslocando até ele de maneira autônoma.

O trabalho deverá atender aos seguintes requisitos:

Encontrar um objeto posicionado dentro do simulador, desde que dentro do limite de visão da câmera;
Segmentar os objetos para extrair os seus atributos;
Extrair o atributo “Ponto da Base” e sua coordenada em pixel;
Encontrar a distância entre este ponto e o robô;
Transformar o ponto do referencial da imagem para o referencial do mundo e utilizar como coordenada de destino;
Controlar o robô de forma que ele chegue em um ponto a 30cm de distância da coordenada da base do objeto (regulação parcial);
Exibir as telas de imagem original, máscara de segmentação e imagem processada;
Informar a distância entre o robô e o objeto de destino, coordenada em pixel do ponto da base e a coordenada do objeto no mundo, ou por impressão no terminal ou por publicação no ROS (info ou publicação de mensagem);
A aplicação deverá ser executada a partir de um arquivo *.launch, que deverá iniciar todos os nós do projeto, caso seja utilizado mais de um nó, assumindo que o simulador já esteja em execução. Os seguintes elementos deverão ser entregues como parte da avaliação:

Pacote ROS completo, com o nomenclatura AT_NOME_SOBRENOME.zip;
Relatório contendo a descrição do(s) nó(s) desenvolvidos, os parâmetros utilizados, tais como limites da máscara, ganhos do controlador, dados do robô e demais configurações utilizadas, bem como os resultados da simulação, como imagens originais e segmentadas e máscara, e a estrutura do seu pacote (árvore de arquivos).
Extra: realizar este mesmo trabalho fazendo a regulação para uma sequência de pelo menos dois ou mais objetos no ambiente. Para este caso, descreva no relatório qual deve ser a sequência que o robô deve seguir durante a operação.

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
