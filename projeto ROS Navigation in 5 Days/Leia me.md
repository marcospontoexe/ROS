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

Para mover o robô pelo ambiente, pode usar o teleop do teclado:
```
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```
Além disso, lembre-se de abrir o rviz e adicionar as exibições adequadas para visualizar o mapa que está a gerar. 

## Salve o Mapa.

Crie um diretório no seu pacote chamado maps e guarde os ficheiros do mapa lá. Use `rosrun map_server map_saver -f nome_do_mapa`.

## Crie um arquivo launch que inicialize o nó map_server.

Você precisará criar um arquivo de lançamento para fornecer o mapa. Como você sabe, isso é feito através do nó map_server. Inicie este arquivo e verifique se ele está realmente fornecendo o mapa.

# Seção 2: Localização 
Nesta seção, você irá localizar o robô utilizando o nó `amcl` e criar um arquivo que registre 3 pontos no mapa para uso posterior. Esta seção possui 5 etapas:

1. Crie um pacote chamado **my_turtlebot_localization** que conterá todos os arquivos relacionados à localização.
2. Crie um arquivo **launch** que inicializará o nó amcl e adicione os parâmetros necessários para configurar adequadamente o nó amcl.
3. Inicialize o nó e verifique se o robô se localiza corretamente no ambiente.
4. Crie uma tabela com três pontos diferentes.
5. Crie um serviço ROS para salvar esses pontos em um arquivo.

## Crie um pacote chamado my_turtlebot_localization

Crie o pacote adicionando rospy como a única dependência.

## Crie um arquivo launch para o nó amcl

Você deverá criar um arquivo de lançamento para o nó amcl e adicionar os parâmetros que achar necessários.

Aqui você pode ver uma lista completa de parâmetros que podem ser configurados no nó amcl: [http://wiki.ros.org/amcl](http://wiki.ros.org/amcl)

Lembre-se de que, antes de definir os parâmetros do nó amcl, será necessário carregar o mapa criado na Seção 1. Para isso, basta incluir o arquivo de lançamento criado na Seção 1 no arquivo de lançamento do nó amcl, a fim de fornecer o mapa para outros nós.

## Inicie o nó e verifique se o robô TurtleBot3 se localiza corretamente no mapa.

Inicie o nó e verifique no rviz se o robô TurtleBot3 se localiza corretamente no mapa.

Para verificar se a localização está funcionando bem, mova o robô pelo ambiente e verifique se a nuvem de partículas continua diminuindo à medida que você move o robô. 

## Crie a tabela de pontos.

Depois de verificar que a localização está funcionando bem, você precisará criar uma tabela com 3 pontos diferentes no ambiente. Para cada ponto, atribua uma etiqueta (com o nome do ponto) junto com suas coordenadas no mapa.

Estes são os 3 pontos que você terá que registrar na tabela:
1. Buscar o primeiro canto (rótulo: corner1).
2. Buscar o segundo canto (rótulo: corner2).
3. Buscar a faixa de pedestres (rótulo: pedestrian).

Você pode acessar as coordenadas de cada posição verificando os tópicos nos quais o nó amcl publica (**/amcl_pose**). Os únicos dados que você realmente precisa salvar são a posição e a orientação. A seguir, você tem uma captura de tela do tópico /amcl_pose:

![amcl_pose](https://github.com/marcospontoexe/ROS/blob/main/imagens/amcl_pose.png)

Crie um arquivo chamado **spots.yaml** e escreva nele os dados de pose que você obteve dos 3 pontos.

## Crie um serviço ROS que salve esses pontos em um arquivo.

Agora, você vai criar um programa ROS que fará o seguinte:

1. Ele lançará um nó chamado **spot_recorder**.
2. Este nó conterá um **servidor de serviço** chamado **/save_spot** que receberá uma string como entrada.
3. Quando este serviço for chamado, ele armazenará as coordenadas atuais do robô (valores de posição e orientação) com um rótulo que será a string fornecida no serviço.
4. Quando for fornecido o comando final na chamada do serviço, o nó escreverá todos os valores armazenados em um arquivo chamado spots.txt.
5. Por fim, o serviço retornará uma mensagem indicando se o arquivo foi salvo corretamente.

Para alcançar isso, vamos dividir em partes menores.

### Mensagem de Serviço
Primeiramente, você terá que determinar o tipo de dados que precisa para sua mensagem de serviço.

1. Determine quais dados de entrada você precisa (request).
2. Determine quais dados você quer que o serviço retorne (response).
Em seguida, veja se já existe uma mensagem construída no sistema que atenda às suas necessidades. Se não houver, então você terá que criar sua própria mensagem personalizada com os dados desejados.

Se for o caso, faça o seguinte. Dentro do pacote que você acabou de criar, crie um novo diretório chamado **srv**. Dentro deste diretório, crie um arquivo chamado **MyServiceMessage.srv** que conterá a definição da sua mensagem de serviço.

Este arquivo pode ser algo assim:
```
# request
string label
---
#response
bool navigation_successfull
string message
```

Agora, você terá que modificar os arquivos **package.xml** e **CMakeLists.txt** do seu pacote para compilar a nova mensagem.

### Código de Serviço
Dentro do diretório src do seu pacote, crie um arquivo chamado **spots_to_file.py**. Dentro desse arquivo, escreva o código necessário para o seu serviço.

### Arquivo de Lançamento
Crie um arquivo de lançamento para o nó que você acabou de criar. Você também pode lançar esse nó no mesmo arquivo de lançamento que você criou para lançar o nó slam_gmapping. A escolha é sua.

### Teste
Usando o teleop do teclado, mova o robô para os 3 diferentes. Em cada um desses pontos, faça uma chamada de serviço para o serviço que você acabou de criar. Na chamada de serviço, forneça a string com o nome que você deseja dar a cada ponto. Por exemplo: `rosservice call /record_spot "label: corner1"`
