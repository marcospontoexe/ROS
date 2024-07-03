# O projeto
Este projeto foi desenvolvido para obter a certificação do curso [ROS Navigation in 5 Days](https://app.theconstruct.ai/courses/57), sendo composto por quatro seções.

* SEÇÃO I: Crie um mapa do ambiente.
* SEÇÃO II: Localize o robô.
* SEÇÃO III: Crie um Sistema de Planejamento de Caminhos (Path Planning).
* SEÇÃO IV: Crie um programa ROS que interaja com o Navigation Stack.

# Seção 1: Mapeamento  
A primeira coisa que você precisa para navegar autonomamente com um robô é um mapa do ambiente. Nesta seção, você praticará como mapear o ambiente ao redor do robô. Seu objetivo é salvar este arquivo de mapa para usá-lo na localização e navegação.

Dividimos isso em 7 passos:

1. Certifique-se de que o robô TurtleBot3 está publicando seus dados de transformação corretamente.
2. Crie um pacote chamado **my_turtlebot_mapping** que conterá todos os arquivos relacionados ao mapeamento.
3. Crie um arquivo launch que iniciará o nó **slam_gmapping** e adicione os parâmetros necessários para configurar adequadamente o nó slam_gmapping.
4. Inicie o nó slam_gmapping e **crie um mapa** do ambiente simulado.
5. **Salve o mapa** que você acabou de criar.
6. Crie um script que **salve mapas automaticamente**.
7. Crie um arquivo launch que **fornecerá o mapa criado** para outros nós.

## Gere arquivos para visualizar a árvore de quadros TF

Você deve obter uma árvore TF como esta:
![frames](https://github.com/marcospontoexe/ROS/blob/main/imagens/frames.png)

## Crie um pacote chamado **my_turtlebot_mapping**.

Crie o pacote, adicionando `rospy` como a única dependência.

## Crie um arquivo launch para o nó gmapping.

Nesta etapa você terá que criar um arquivo launch para o nó slam_gmapping e adicionar os parâmetros que você acha que precisa configurar.

Aqui pode ver uma lista completa de parâmetros que pode configurar para o nó slam_gmapping: [Parâmetros slam_gmapping](https://docs.ros.org/en/hydro/api/gmapping/html/)

## Launch o nó usando o arquivo launch que acabou de criar e crie um mapa do ambiente.

Para mover o robô pelo ambiente, pode usar o teleop do teclado. Além disso, lembre-se de abrir o rviz e adicionar as exibições adequadas para visualizar o mapa que está a gerar. 

## Salve o Mapa.

Crie um diretório no seu pacote chamado maps e guarde os ficheiros do mapa lá. 

## Crie um arquivo launch que inicialize o nó map_server.

Você precisará criar um arquivo de lançamento para fornecer o mapa. Como você sabe, isso é feito através do nó map_server. Inicie este arquivo e verifique se ele está realmente fornecendo o mapa.
