# ROS
É um framework de código aberto utilizado para o desenvolvimento de software para robôs. Embora seja chamado de "sistema operacional", o ROS não é um sistema operacional no sentido tradicional, mas sim um conjunto de ferramentas, bibliotecas e convenções que visam simplificar o desenvolvimento de software para robótica. O ROS fornece funcionalidades como gerenciamento de dispositivos, controle de hardware, troca de mensagens entre processos e visualização de dados, facilitando a criação de sistemas complexos de robótica. Ele é amplamente utilizado na comunidade de pesquisa e desenvolvimento de robótica devido à sua flexibilidade e modularidade.

##  Launch files
Um programa ROS é executado usando alguns arquivos especiais chamados launch files, que ficam dentro dos pacotes. A estrutura do comando roslaunch é a seguinte: `roslaunch <package_name> <launch_file>`.
* package_name: Especifica o nome do pacote ROS contendo o launch files.
* launch_file:  E o nome do launch file em si.(que está armazenado dentro do pacote).

### Pacote
O ROS utiliza pacotes para organizar seus programas. Você pode pensar em um pacote como todos os arquivos que um programa ROS específico contém; 
* todos os seus arquivos cpp,
* arquivos python,
* arquivos de configuração,
* arquivos de compilação,
* arquivos de lançamento (launch files),
* arquivos de parâmetros.

Todos esses arquivos no pacote são organizados com a seguinte estrutura:
* pasta launch: Contém arquivos de lançamento.
* pasta src: Arquivos de origem (cpp, python).
* CMakeLists.txt: Lista de regras do cmake para compilação.
* package.xml: Informações do pacote e dependências.

Para acessar qualquer pacote do ROS, o ROS oferece um comando chamado **roscd**. Ao digitar: `roscd <package_name>`, ele o levará ao diretório onde o pacote package_name está localizado.

### Como a launch file funciona?
No arqivo launch, há algumas tags extras para definir parâmetros e redirecionamentos. A tag **param** define um parâmetro no Servidor de Parâmetros, de onde os nós obtêm parâmetros. Muitos nós utilizam parâmetros para evitar modificar o código-fonte, a baixo você pode ver como eles são adicionados.

    ```
    <launch>
      <!-- turtlebot_teleop_key already has its own built in velocity smoother -->
      <node pkg="turtlebot_teleop" type="turtlebot_teleop_key.py" name="turtlebot_teleop_keyboard"  output="screen">
        <param name="scale_linear" value="0.5" type="double"/>
        <param name="scale_angular" value="1.5" type="double"/>
        <remap from="turtlebot_teleop_keyboard/cmd_vel" to="/cmd_vel"/>   <!-- cmd_vel_mux/input/teleop"/-->
      </node>
    </launch>
    ```

A tag **remap** redireciona mensagens de um tópico para outro. No caso a cima, o nó de teleop publica por padrão em `turtlebot_teleop_keyboard/cmd_vel`. Queremos que ele publique em `cmd_vel`, então a tag é adicionada.

Todos os launch file estão contidos em uma tag **launch**. Dentro dessa tag, você pode ver uma tag **node**, onde especificamos os seguintes parâmetros:
* pkg="nome_do_pacote": Nome do pacote que contém o código do programa ROS a ser executado.
* type="nome_do_arquivo_python.py": Nome do arquivo do programa que queremos executar.
* name="nome_do_nó": Nome do nó ROS que lançará nosso arquivo Python.
* output="tipo_de_saída": Através de qual canal você imprimirá a saída do arquivo Python.

### Criando um pacote ROS
Quando queremos criar pacotes, precisamos trabalhar em um espaço de trabalho ROS, conhecido como espaço de trabalho Catkin (**catkin_ws**). Para fazer isso;
1. Digite **roscd** no terminal. Você verá que é direcionado para um diretório `catkin_ws/devel`.
2. Digitar `cd ..` para subir um diretório. Você deverá terminar aqui em `/home/user/catkin_ws`.
3. Dentro deste espaço de trabalho, há um diretório chamado `src`. Esta pasta conterá todos os pacotes criados. Sempre que você quiser criar um novo pacote, é necessário estar neste diretório (`catkin_ws/src`). Digite no seu terminal `cd src` para mover-se para o diretório de origem.
4. Agora estamos prontos para criar nosso primeiro pacote! Para criar um pacote, digite no seu terminal: `catkin_create_pkg nome_do_pacote package_dependencies`. O **nome_do_pacote** é o nome do pacote que você deseja criar, e o **package_dependencies** são os nomes de outros pacotes ROS dos quais seu pacote depende (rospy, std_msgs...).
5. Isso criará dentro do nosso diretório src um novo pacote com alguns arquivos e diretórios (src, CMakeLists.txt e package.xml).

Para verificar se nosso pacote foi criado com sucesso, podemos usar alguns comandos ROS relacionados a pacotes. Por exemplo, vamos digitar:
* `rospack list | grep nome_do_pacote`: Para filtrar, de todos os pacotes localizados no sistema ROS, o pacote chamado "nome_do_pacote".
* `roscd nome_do_pacote`: Leva você à localização no disco rígido do pacote chamado "nome_do_pacote".

Dentro do pacote deve conter;

7. Um diretório chamado **src**: dentro desse diretório deve ficar o arquivo python. Verifique se o arquivo tem permissão de execusão.
   * A primeira linha do arquivo deve conter **#! /usr/bin/env python **.
   * Deve importar a bilioteca **rospy** (import rospy).
     
8. Um diretório chamado **launch**: Dentro desse diretório deve conter um arquivo de extensão **.launch**. O arquivo louch deve conter algo semelhante com o que foi descrito no tópico a cima; "Como a launch file funciona?".

Para executar o programa criado no pacote, execute o comando `roslaunch nome_do_pacote nome_package_launch_file.launch`.

Às vezes, o ROS não detectará um novo pacote quando você acabou de criá-lo, então você não poderá fazer um `roslaunch`. Nesse caso, você pode forçar o ROS a atualizar sua lista de pacotes com o comando: `rospack profile`.

[Veja nesse diretório](https://github.com/marcospontoexe/ROS/tree/main/Pacotes/exemplos) exemplo de pacotes criados.

### Compilando um pacote
Quando você cria um pacote, geralmente precisará compilá-lo para fazê-lo funcionar. Existem diferentes métodos que podem ser usados para compilar seus pacotes ROS, o mais comum: **catkin_make**.

Este comando irá compilar todo o seu diretório src, e ele precisa ser executado no seu diretório catkin_ws para funcionar (`cd ~/catkin_ws`). Se você tentar compilar a partir de outro diretório, não funcionará.

Depois de compilar, também é muito importante "sourcer" (fornecer) o seu espaço de trabalho. Isso garantirá que o ROS sempre obtenha as últimas alterações feitas no seu espaço de trabalho: `source devel/setup.bash`.

Às vezes (por exemplo, em projetos grandes), você não vai querer compilar todos os seus pacotes, mas apenas aquele(s) onde você fez alterações. Você pode fazer isso com o seguinte comando: `catkin_make --only-pkg-with-deps nome_do_pacote`.

## Os nós do ROS
Nós do ROS são basicamente programas feitos no ROS. O comando ROS para ver quais nós estão realmente em execução em um computador é: `rosnode list`.

Para ver informações sobre um nó, podemos usar o comando: `rosnode info nome_do_nó`. 

**rosrun** permite executar um nó de forma simples, sem a necessidade de especificar o caminho completo para o executável. Você só precisa fornecer o nome do pacote e o nome do nó: `rosrun nome_do_pacote nome_do_nó`. É possível passar argumentos adicionais ao nó que está sendo executado. Por exemplo: `rosrun turtlesim turtle_teleop_key _param:=value`, "_param:=value" define um parâmetro ROS específico que será passado ao nó.

## Parameter Server
Um servidor de parâmetros é um dicionário que o ROS usa para armazenar parâmetros. Esses parâmetros podem ser usados pelos nós em tempo de execução e são normalmente usados para dados estáticos, como parâmetros de configuração.
* Para obter uma lista desses parâmetros, você pode digitar: `rosparam list`.
* Para obter o valor de um parâmetro específico, você pode digitar: `rosparam get nome_do_parâmetro`.
* E para definir um valor para um parâmetro, você pode digitar: `rosparam set nome_do_parâmetro valor_do_parâmetro`.

## ROS Core
Para que tudo isso funcione, precisamos ter um roscore em execução. O roscore é o processo principal que gerencia todo o sistema ROS. Você sempre precisa ter um roscore em execução para trabalhar com o ROS. O comando que inicia um roscore é: `roscore`.

Veja um diagrama de um Roscore.

![roscore diagrama](https://github.com/marcospontoexe/ROS/blob/main/imagens/roscore.jpg).

## Variáveis de ambiente
O ROS usa um conjunto de variáveis de ambiente do sistema Linux para funcionar corretamente. Você pode verificar essas variáveis digitando: `export | grep ROS`.

As variáveis mais importantes são;
**ROS_MASTER_URI**: Contém o URL onde o ROS Core está sendo executado. Normalmente, é o próprio computador (localhost).
**ROS_PACKAGE_PATH**: Contém os caminhos no seu disco rígido onde o ROS possui pacotes.

## Python POO
Para mais detalhes sobre orientação a objetos, acesse meu repósitório [POO (Object-Oriented Programming) no Python](https://github.com/marcospontoexe/Python/blob/main/README.md).

[Veja esse pacote, "poo"](https://github.com/marcospontoexe/ROS/tree/main/Pacotes/exemplos/poo), uma classe chamada "MoveBB8", implementada no código "bb8_move_circle_class.py," foi criada para mover o robo bb8 em circulo. Essa classe é importada pelo código "bb8_move_circle_service_server.py", que inicia o serviço "/move_bb8_in_circle", instanciando um objeto da classe MoveBB8 para fazer o robo andar em circulo durante um tempo fornecido pela variável de Resquest "duration". Quando o serviço move_bb8_in_circle for chamado (`rosservice call /move_bb8_in_circle [TAB]+[TAB]`) o robo começa a se mover em circulo por um determinado tempo.  

```
#! /usr/bin/env python

import rospy
from std_srvs.srv import Empty, EmptyResponse 
from bb8_move_circle_class import MoveBB8

def my_callback(request):
    rospy.loginfo("The Service move_bb8_in_circle has been called")
    movebb8_object = MoveBB8()
    movebb8_object.move_bb8()
    rospy.loginfo("Finished service move_bb8_in_circle")
    return EmptyResponse() 

rospy.init_node('service_move_bb8_in_circle_server') 
my_service = rospy.Service('/move_bb8_in_circle', Empty , my_callback)
rospy.loginfo("Service /move_bb8_in_circle Ready")
rospy.spin() # keep the service open.
```

## Tópicos
O ROS lida quase que inteiramente com suas comunicações (mensagens) por meio de tópicos. Até mesmo sistemas de comunicação mais complexos, como **serviços** ou **ações**, dependem, em última análise, de tópicos. É por isso que eles são tão importantes! Através dos tópicos do ROS, você será capaz, por exemplo, de se comunicar com seu robô para fazê-lo se mover, ler as leituras dos sensores do seu robô e muito mais.

Um tópico é um canal, onde outros nós ROS podem tanto publicar quanto ler informações (mensagens) para que eles possam se comunicar.

Você pode solicitar informações sobre um tópico digitando `rostopic info nome_do_tópico`. A saída indica:
* Type: Tipo de informação publicada, 
* Publisher: Nó que está publicando essa informação,
* Subscribers: Se há um nó ouvindo essa informação.

Alguns comando úteis:
* Para obter uma lista de tópicos disponíveis em um sistema ROS: `rostopic list`.
* O comando `rostopic echo /nome_do_tópico` mostra as informações que estão sendo publicadas em um tópico.
* Para ler apenas a última mensagem publicada em um tópico com o comando: `rostopic echo nome_do_tópico -n1`.
* Para obter informações sobre um determinado tópico: `rostopic info nome_do_tópico`.
* Você pode verificar as diferentes opções que o comando rostopic possui usando o próximo comando: `rostopic -h`.

### Mensagens
Como você pode ter notado, os tópicos lidam com informações por meio de mensagens. Existem muitos tipos diferentes de mensagens. Você até pode criar suas próprias mensagens, mas é recomendável usar as mensagens padrão do ROS sempre que possível. Veja a baixo os tipos de menssagens padrões do ROS

![tipo de mensagens](https://github.com/marcospontoexe/ROS/blob/main/imagens/tipo%20de%20mensagens.png)

As mensagens são definidas em arquivos **.msg**, que estão localizados dentro de um diretório chamado **msg** de um pacote.

Para obter informações sobre uma mensagem, use o comando `rosmsg show tipo_da_mensagem`.

#### Criando um tipo de mensagem
Para criar uma nova mensagem, você precisará seguir os seguintes passos:
1. Crie um diretório chamado **msg** dentro do seu pacote.
2. Dentro deste diretório, crie um arquivo chamado **Nome_da_sua_mensagem.msg**.
3. Modifique o arquivo CMakeLists.txt. 
4. Modifique o arquivo package.xml.
5. Compile.
6. Use no código.

**Atenção** existe um problema no ROS que pode causar problemas ao importar mensagens do diretório msg. Se o nome do seu pacote for o mesmo que o do arquivo Python que importa a mensagem, isso causará um erro dizendo que não encontra o elemento msg. Isso ocorre devido à forma como o Python funciona. Portanto, você deve ter cuidado para não nomear o arquivo Python exatamente igual ao seu pacote pai. Isso causará um erro de importação porque ele tentará importar a mensagem do arquivo my_package.py, de um diretório .msg que não existe.

Por exemplo, vamos criar uma mensagem que indica a idade, com anos, meses e dias.

##### Criando um arquivo.msg
Dentro do diretório "msg" crie um arquivo com extensão **.msg** (para este exemplo: Age.msg), e coloque as variáveis usadas nesse arquivo, para o nosso exemplo veja a baixo:
```
float32 years
float32 months
float32 days
```

##### Modificando o arquivo **CMakeLists.txt**
Você terá que editar quatro funções dentro do arquivo CMakeLists.txt:
1. find_package(): Aqui é onde todos os pacotes necessários para COMPILAR as mensagens dos tópicos, serviços e ações são inseridos. No package.xml, você precisa declará-los como build_depend.
    * DICA 1: Se você abrir o arquivo CMakeLists.txt no seu IDE, verá que quase todo o arquivo está comentado. Isso inclui algumas das linhas que você terá que modificar. Em vez de copiar e colar as linhas abaixo, encontre os equivalentes no arquivo e descomente-os, e depois adicione as partes que estão faltando.
    ```
      find_package(catkin REQUIRED COMPONENTS
        rospy
        std_msgs
        message_generation   # Adicione message_generation aqui, depois dos outros pacotes
      )
    ```
2. add_message_files(): Essa função inclui todas as mensagens deste pacote (na pasta msg) para serem compiladas. O arquivo deve se parecer com isso.
    ```
    add_message_files(
      FILES
      Age.msg
    ) # Não esqueça de DESCOMENTAR essa função
    ```
3. generate_messages(): Aqui é onde os pacotes necessários para a compilação das mensagens são importados.
    ```
    generate_messages(
      DEPENDENCIES
      std_msgs
    ) # Não esqueça de DESCOMENTAR essa função
    ```
4. catkin_package(): Aqui são listados todos os pacotes que serão necessários para alguém que execute algo do seu pacote. Todos os pacotes listados aqui devem estar no package.xml como **exec_depend**.
    ```
    catkin_package(
      CATKIN_DEPENDS rospy message_runtime   
    )
    ```

##### Modificando o arquivo **package.xml**
Basta adicionar estas 3 linhas detro da tag "package" do arquivo package.xml.
```
<build_depend>message_generation</build_depend> 

<build_export_depend>message_runtime</build_export_depend>
<exec_depend>message_runtime</exec_depend>
```

##### Compilando 
Agora você precisa compilar as mensagens. Para fazer isso, digite no terminal:
1. `roscd; cd ..`
2. `catkin_make --only-pkg-with-deps nome_do_pacote`
3. `source devel/setup.bash`, Isso executa este arquivo bash que configura, entre outras coisas, as novas mensagens geradas criadas através do catkin_make. Se você não fizer isso, pode ocorrer um erro de importação em Python, dizendo que não encontra a mensagem gerada. **Execute este comando no terminal em que for usar**.

Para verificar se sua mensagem foi criada com sucesso, digite em seu terminal `rosmsg show nome_da_mensagem_criada` (para esse exemplo `rosmsg show Age`). Se a estrutura da mensagem aparecer, significa que sua mensagem foi criada com sucesso e está pronta para ser usada em seus programas ROS.

[Veja nesse nó](https://github.com/marcospontoexe/ROS/tree/main/Pacotes/exemplos/message) um mensagem do tipo float32 criada para indicar a idade, com anos, meses e dias.

### Publishers
Um publisher é um nó que fica publicando uma mensagem em um tópico.

Este comando publicará a mensagem que você especificar com o valor que você especificar, no tópico que você especificar: `rostopic pub topic_name message_type value` (por exemplo `rostopic pub /counter std_msgs/Int32 5`).

[Veja nesse exemplo](https://github.com/marcospontoexe/ROS/tree/main/Pacotes/exemplos/my_publisher_example_pkg) um nó publisher que fica publicando uma menssagens do tipo **Int32*.

[Veja nesse exemplo](https://github.com/marcospontoexe/ROS/tree/main/Pacotes/exemplos/my_publisher_move_pkg) um nó publisher criado para mover um robo em circulo. O programa em Python cria um nó (move_robot_node) para publicar uma menssagem (Twist), através do tópico **cmd_vel** para mover o robo em circulo. 

### Subscriber
Um Subscriber é um nó que lê informações de um tópico.

[Veja nesse nó do tipo subscriber](https://github.com/marcospontoexe/ROS/tree/main/Pacotes/exemplos/my_subscriber_example_pkg) como receber uma mensagem do tipo **Int32** pelo tópico **counter**.

[Veja aqui](https://github.com/marcospontoexe/ROS/tree/main/Pacotes/exemplos/my_subscriber_odometry_pkg) um nó subscriber chamado de "odometry_node" que recebe uma mensagem do tipo **Odometry** pelo tópico **odom**.

[Veja o quiz](https://github.com/marcospontoexe/ROS/tree/main/Pacotes/exemplos/topics_quiz) proposto no módulo cinco do curso ["ROS basics in 5 dyas (python)"](https://app.theconstruct.ai/courses/55). O quiz propoem criar um nó para guiar o robo.
1. Crie um Publisher que escreva no tópico /cmd_vel para movimentar o robô.
2. Crie um Subscriber que leia do tópico /kobuki/laser/scan. Este é o tópico onde o laser publica seus dados.
3. Dependendo das leituras que você receber do tópico do laser, você terá que alterar os dados que está enviando para o tópico /cmd_vel para evitar a parede. Isso significa usar os valores do laser para decidir.

Seu programa deve seguir a seguinte lógica:
1. Se a leitura do laser à frente do robô for superior a 1 metro, o robô avançará.
2. Se a leitura do laser à frente do robô for inferior a 1 metro, o robô virará à esquerda.
3. Se a leitura do laser ao lado direito do robô for inferior a 1 metro, o robô virará à esquerda.
4. Se a leitura do laser ao lado esquerdo do robô for inferior a 1 metro, o robô virará à direita.

## Serviços
Os serviços permitem que você desenvolva uma funcionalidade específica para seu robô e depois a disponibilize para que qualquer pessoa possa chamá-la. Por exemplo, você poderia criar um serviço que faça seu robô se mover por um período específico de tempo e depois parar.

Os serviços são um pouco mais complexos do que os tópicos, pois são estruturados em duas partes. De um lado, você tem o **Servidor de Serviço**, que fornece a funcionalidade para qualquer pessoa que queira usá-la (chamá-la). Do outro lado, você tem o **Cliente de Serviço**, que é aquele que faz a chamada/solicitação da funcionalidade do serviço.

O **servidor** de serviço cria um serviço para enviar uma mensagem de Response através desse um serviço.

O **cliente** de serviço, ao conectar-se a um servidor de serviço, envia uma menssagem de Resquest para o serviço conectado.

O serviço deve estar em funcionamento antes que você possa chamá-lo. Portanto, certifique-se de ter iniciado o serviço antes de chamá-lo. 

Para visualizar a lista de serviços ativos digite o comando `rosservice list`.

Para ter informações sobre um serviço digite `rosservice info /name_of_your_service`. Este comando retorna quatro dados:
* Node: Indica o nó que fornece (criou) esse serviço.
* URI: O URI onde o serviço pode ser acessado.
* Type: Refere-se ao tipo de mensagem usada por este serviço. Tem a mesma estrutura que os tópicos. É sempre composto por pacote_onde_a_mensagem_do_serviço_é_definida / Nome_do_Arquivo_onde_a_mensagem_do_Serviço_é_definida.
* Args: Aqui você pode encontrar os argumentos que este serviço aceita quando é chamado (valor, path, comando...). 

Você pode chamar um serviço manualmente a partir do terminal. Isso é muito útil para testes e para ter uma ideia básica de como o serviço funciona: `rosservice call /the_service_name TAB+TAB`. Quando você pressiona [TAB]+[TAB] rapidamente, um elemento extra aparece (**"nome_mensagem: '' "**), coloque o valor da mensagem desejada a ser executada pelo serviço dentro das aspas simples.

[Veja nesse exemplo](https://github.com/marcospontoexe/ROS/tree/main/Pacotes/exemplos/my_service_client_example_pkg) como enviar um Request de um serviço do tipo cliente através de uma mensagem de serviço. Para enviar a mensagem de serviço, o serviço deve estar em execução. Execute o serviço com o comando: `roslaunch trajectory_by_name start_service.launch`.

[Esse exemplo](https://github.com/marcospontoexe/ROS/tree/main/Pacotes/exemplos/move_arm) inicia o servidor de serviço "execute_trajectory", que é inicializado pela launch "start_service", que está no pacote "iri_wam_reproduce_trajectory". Ao iniciar o serviço "execute_trajectory", o arquivo python realiza uma conexão com "execute_trajectory" e inicia uma mensagem (Request) de serviço do tipo "ExecTraj", do pacote "iri_wam_reproduce_trajectory", para fazer o braço do robô se mover seguindo uma trajetória especificada em um arquivo, que é solicitado por esse Request do serviço "execute_trajectory".

[Veja nesse exemplo](https://github.com/marcospontoexe/ROS/tree/main/Pacotes/exemplos/my_service_server_example_pkg) um **servidor de serviços** chamado "my_service" que recebe uma mensagem de serviço *Request* do tipo "Empty", e envia uma mensagem de serviço *Response* do tipo EmptyResponse. Para ver o Response do servidor de serviços, digite o comando `rosservice call /my_service "{}"` em outro terminal.

[Nesse exemplo](https://github.com/marcospontoexe/ROS/tree/main/Pacotes/exemplos/move_around_server_pkg) a launch "start_bb8_move_in_circle_service_server" inicia um servidor de serviço chamado "move_bb8_in_circle", que aceita uma mensagem de serviço vazia e ativa o movimento circular do robo, através da mensagem "Twist" enviada para o tópico "cmd_vel". A launch "call_bb8_move_in_circle_service_server" inicia um cliente de serviço que chama o serviço "move_bb8_in_circle" iniciado pela launch "start_bb8_move_in_circle_service_server". Para o cliente de serviço rodar é necessário ter iniado o servidor de serviço.


### Mensagens de um serviço
Arquivos de mensagem de serviço têm a extensão **.srv** e são definidos dentro de um diretório chamado **srv**.

Para explorar a estrutura de uma mensagem de serviço use o comando comando `rossrv show name_of_the_package/Name_of_Service_message`. Name_of_Service_message é o Nome do arquivo onde a mensagem do serviço é definida, mostrado pelo comando (`rosservice info /name_of_your_service`). Na imagens a baixo é mostrado o resutado dos dois comandos.

![estrutura de uma mensagem de serviço](https://github.com/marcospontoexe/ROS/blob/main/imagens/mensagem%20de%20um%20servi%C3%A7o.png)

As mensagens de um serviço tem duas partes **Request** e **Response**. Na figura a cima, request contém uma string chamada "traj_name" e response é composta por um booleano chamado "success" e uma string chamada "status_message".

Request é a parte da mensagem que significa quais variáveis você terá que passar para o Servidor de Serviço para que ele seja capaz de concluir sua tarefa.

Response é a parte da mensagem de serviço que define como o seu serviço responderá após concluir sua funcionalidade. Se, por exemplo, ele retornará uma string com uma mensagem específica dizendo que tudo correu bem, ou se não retornará nada, etc...

Sempre que uma mensagem de serviço é compilada, é retornado três objetos:
1. A própria mensagem de serviço: É usado para criar uma conexão com o servidor de serviço, como demonstrado no exemplo anterior: ` traj_by_name_service = rospy.ServiceProxy('/trajectory_by_name', TrajByName)`.
2. MyServiceMessageRequest: Este é o objeto usado para criar uma solicitação a ser enviada ao servidor. Portanto, este objeto é usado para enviar uma solicitação para o servidor de serviço, como demonstrado no exemplo anterior:
    ```
    # Create an object of type TrajByNameRequest
    traj_by_name_object = TrajByNameRequest()
    # Fill the variable traj_name of this object with the desired value
    traj_by_name_object.traj_name = "release_food"
    # Send through the connection the name of the trajectory to be executed by the robot
    result = traj_by_name_service(traj_by_name_object)
    ```
3. MyServiceMessageResponse: Este é o objeto usado para enviar uma resposta do servidor de volta para o cliente, sempre que o serviço termina. 

### Criando um mensagem de serviço
Você pode colocar quantas variáveis precisar, de qualquer tipo suportado pelo ROS.

1. Crie uma pasta **srv** dentro do seu pacote. Em seguida, dentro dessa pasta srv, crie um arquivo chamado **MyCustomServiceMessage.srv**. 
2. Dentro de MyCustomServiceMessage.srv coloque suas variáveis de Request e Response, como no exemplo a baixo:
      ```
      int32 duration    # The time (in seconds) during which BB-8 will keep moving in circles
      ---
      bool success      # Did it achieve it?
      ```
3. Edite o arquivo "CMakeLists.txt".
4. Edite o arquivo "package.xml".

#### Editando o arquivo CMakeLists.txt.
Você terá que editar quatro funções dentro do arquivo CMakeLists.txt: 

1. **find_package()**: Todos os pacotes necessários para COMPILAR as mensagens de tópicos, serviços e ações vão aqui. Ele está apenas obtendo seus caminhos e não realmente importando-os para serem usados na compilação. Os mesmos pacotes que você escreve aqui irão para o package.xml, declarando-os como build_depend.
      ```
      find_package(catkin REQUIRED COMPONENTS
      std_msgs
      message_generation
      )
      ```
2. **add_service_files()**: Esta função contém uma lista de todas as mensagens de serviço definidas neste pacote (definidas na pasta srv). Por exemplo:
      ```
      add_service_files(
      FILES
      MyCustomServiceMessage.srv
      )
      ```
3. **generate_messages()**: Aqui é onde os pacotes necessários para a compilação das mensagens de serviço são importados.
      ```
      generate_messages(
      DEPENDENCIES
      std_msgs
      )
      ```  
4. **catkin_package()**: Liste aqui todos os pacotes que serão necessários para quem executar algo do seu pacote. Todos os pacotes mencionados aqui devem estar no arquivo package.xml como <exec_depend>.
      ```
      catkin_package(
      CATKIN_DEPENDS
      rospy
      )
      ```

Depois de terminar de editar o arquivo, tera algo parecido com isso:

```
cmake_minimum_required(VERSION 2.8.3)
project(my_custom_srv_msg_pkg)

## Here is where all the packages needed to COMPILE the messages of topics, services and actions go.
## It's only getting its paths, and not really importing them to be used in the compilation.
## It's only for further functions in CMakeLists.txt to be able to find those packages.
## In package.xml you have to state them as build
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)

## Generate services in the 'srv' folder
## In this function will be all the action messages of this package ( in the action folder ) to be compiled.
## You can state that it gets all the actions inside the action directory: DIRECTORY action
## Or just the action messages stated explicitly: FILES my_custom_action.action
## In your case you only need to do one of two things, as you wish.
add_service_files(
  FILES
  MyCustomServiceMessage.srv
)

## Here is where the packages needed for the action messages compilation are imported.
generate_messages(
  DEPENDENCIES
  std_msgs
)

## State here all the packages that will be needed by someone that executes something from your package.
## All the packages stated here must be in the package.xml as exec_depend
catkin_package(
  CATKIN_DEPENDS rospy
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)
```

#### Editando o arquivo package.xml
Adicione todos os pacotes necessários para compilar as mensagens. Neste caso, você só precisa adicionar o **message_generation**. Você terá que importar esses pacotes como <build_depend>. Por outro lado, se você precisa de um pacote para a execução dos programas dentro do seu pacote, você terá que importar esses pacotes como <exec_depend>.

Neste caso, você só precisará adicionar estas 3 linhas ao seu arquivo package.xml:

  ```
  <build_depend>message_generation</build_depend>
  <build_export_depend>message_runtime</build_export_depend>
  <exec_depend>message_runtime</exec_depend>
  ```

Depois de terminar, compile (**`catkin_make --only-pkg-with-deps nome_do_pacote`**) seu pacote e faça o sourcement (**`source devel/setup.bash`**) das mensagens recém-geradas.

Para verificar se você tem o novo serviço de mensagem no seu sistema, pronto para ser utilizado, digite o comando `rossrv list | grep MyCustomServiceMessage`.

[Veja no pacote "services_quiz"](https://github.com/marcospontoexe/ROS/tree/main/Pacotes/exemplos/services_quiz) uma mensagem de serviço chamada de "BB8CustomServiceMessage.srv" que tem duas variáreis de Resqueste (float64 side, int32 repetitions), e uma variável de Response (bool success). A launche "start_bb8_move_custom_service_server.launch" inicia o serviço **/move_bb8_in_square_custom** através do servidor de serviço "bb8_move_custom_service_server.py". A launch "call_bb8_move_in_square_custom_service_server.launch" inicia o cliente de serviço "bb8_move_custom_service_client.py", que envia a mensagens de Requet para o servidor, fazendo o robo executar um movimento quadrado.

## Ações
O ROS também fornece ações. As ações são semelhantes aos serviços, no sentido de que também permitem que você codifique uma funcionalidade para o seu robô e, em seguida, a disponibilize para que qualquer pessoa possa chamá-la. A principal diferença entre ações e serviços é que, ao chamar um serviço, o robô precisa esperar até que o serviço tenha terminado antes de fazer algo mais. Por outro lado, ao chamar uma ação, o seu robô ainda pode continuar fazendo outra coisa enquanto executa a ação.

Há outras diferenças, como uma ação permitindo que você forneça feedback enquanto a ação está sendo realizada.

O nó que **fornece a funcionalidade** precisa conter um **servidor de ações**. O servidor de ações permite que outros nós chamem essa funcionalidade de ação.

O nó que **chama a funcionalidade** precisa conter um **cliente de ações**. O cliente de ações permite que um nó se conecte ao servidor de ações de outro nó.

Para descobrir quais ações estão disponíveis em um robô, executar `rostopic list`. O comando retornara 5 tópicos com o mesmo nome base, cada um com os **sub-tópicos cancel, feedback, goal, result e status**. São as mensagens usadas para se **comunicar com o Servidor de Ações**.

Chamar um servidor de ações significa enviar uma mensagem para ele. Da mesma forma que acontece com tópicos e serviços, tudo funciona passando mensagens:
* A **mensagem de um tópico** é composta por uma única parte: a informação que o tópico fornece.
* A **mensagem de um serviço** possui duas partes: a solicitação (Request) e a resposta (Response).
* A **mensagem de um servidor** de ações é dividida em três partes: o objetivo (goal), o resultado (result) e o feedback. E cada parte pode conter mais de uma variável.

A imagem a baixo mostra um exemplo de uma mensagem de um servidor de ações, composta por três partes.

![mensagems de servidor de ações](https://github.com/marcospontoexe/ROS/blob/main/imagens/mensagens%20action.png)

**goal**: Consiste em uma variável chamada "nseconds" do tipo Int32. O tipo Int32 é um tipo de mensagem padrão do ROS, portanto, pode ser encontrado no pacote std_msgs. Por ser um pacote padrão do ROS, não é necessário indicar o pacote onde Int32 pode ser encontrado.

**result**: Consiste em uma variável chamada "allPictures", que é um array do tipo CompressedImage[], encontrado no pacote "sensor_msgs".

**feedback**: Consiste em uma variável chamada "lastImage" do tipo CompressedImage[], encontrado no pacote "sensor_msgs".

Todas as mensagens de ação utilizadas são definidas no **diretório action** do pacote correspondente.

Devido ao fato de que chamar um servidor de ações não interrompe sua thread, os servidores de ações fornecem uma mensagem chamada **feedback**. O feedback é uma mensagem que o servidor de ações gera de tempos em tempos para indicar como está progredindo a ação (informando ao chamador o status da ação solicitada). Essa mensagem é **gerada enquanto a ação está em andamento**.

[Veja como chamar uma ação](https://github.com/marcospontoexe/ROS/tree/main/Pacotes/exemplos/my_action_client_example_pkg) com esse cliente de ação que chama o `ardrone_action_server` e o faz tirar fotos por 10 segundos. É preciso ter o `roslaunch ardrone_as action_server.launch` em execução. Se o arquivo de mensagem de ação se chamasse **Ardrone.action**, então o tipo de mensagem de ação que você deve especificar é **ArdroneAction**, e o tipo de mensagem **goal** que você deve especificar é **ArdroneGoal()**.

### Como realizar outras tarefas enquanto a Ação está em progresso
Os objetos **SimpleActionClient** têm duas funções que podem ser usadas para saber se a ação que está sendo realizada foi concluída e como:
1. **wait_for_result():** Esta função é bastante simples. Quando chamada, ela espera até que a ação seja concluída e retorna um valor verdadeiro. Como você pode ver, ela é inútil se você deseja realizar outras tarefas em paralelo, porque o programa ficará parado até que a ação seja concluída.
2. **get_state():** Esta função é muito mais interessante. Quando chamada, ela retorna um inteiro que indica em qual estado está a ação à qual o objeto SimpleActionClient está conectado.
```
  0 ==> PENDING
  1 ==> ACTIVE
  2 ==> PREEMPTED
  3 ==> SUCCEEDED
  4 ==> ABORTED
  5 ==> REJECTED
  6 ==> PREEMPTING
  7 ==> RECALLING
  8 ==> RECALLED
  9 ==> LOST
```

Isso permite que você crie um laço `while` que verifica se o valor retornado por `get_state()` indica sucesso, ainda está processando, foi chamado novamente, foi abortado, etc. Isso permite que você verifique o status do objetivo da ação, enquanto ainda é capaz de realizar outras tarefas nesse ínterim.

[Veja nesse exemplo](https://github.com/marcospontoexe/ROS/tree/main/Pacotes/exemplos/my_action_client_getstate_pkg) um nó chamado "example_with_waitforresult_action_client_node" que espera a ação terminar para fazer algo, e um nó chamado "example_no_waitforresult_action_client_node" que faz algo enquanto a ação ainda está sendo executada. Para executar esses nós, o cliente servidor deve estar rodando (roslaunch ardrone_as action_server.launch).

[Veja nesse pacote criado (client_action_move_drone_around_pkg)](https://github.com/marcospontoexe/ROS/tree/main/Pacotes/exemplos/client_action_move_drone_around_pkg), um cliente de ações que faz o quadricóptero se mover em circulo enquanto o servidor de ações foi chamado (para tirar fotos enquanto o robô está se movendo), parando o movimento do quadricóptero quando a última foto for tirada (o servidor de ações tiver terminado). Você deve enviar comandos de movimento enquanto espera até que o resultado seja recebido, criando um loop que envia comandos ao mesmo tempo em que verifica a conclusão. Para poder enviar comandos enquanto a ação está em andamento, você precisa usar a função **get_state()** da **SimpleActionClient**. Para executar esse cliente o servidor deve estar rodando (`roslaunch ardrone_as action_server.launch`).

### Antecipando um objetivo (Preempting a goal)
Acontece que você pode cancelar um objetivo previamente enviado para um servidor de ações antes de sua conclusão.
Cancelar um objetivo enquanto ele está sendo executado é chamado de antecipar um objetivo.

Para antecipar um objetivo, você envia o **cancel_goal** para o servidor através da conexão do cliente: `client.cancel_goal()`.

[Veja nesse exenplo](https://github.com/marcospontoexe/ROS/tree/main/Pacotes/exemplos/cancel_action_pkg) um programa que conta até 2 e, em seguida, cancela o objetivo. Isso aciona o servidor para finalizar o objetivo e, portanto, a função `get_state()` retorna o valor DONE (2).

Existe um problema conhecido no ROS com Ações. Ele emite um aviso quando a conexão é interrompida. Isso geralmente ocorre ao cancelar um objetivo ou ao encerrar um programa que contenha um objeto cliente. O aviso é emitido no lado do servidor, porém não causa nenhum efeito no programa:

```
[WARN] Inbound TCP/IP connection failed: connection from sender 
terminated before handshake header received. 0 bytes were received. 
Please check sender for additional details.
```

### Como funciona a comunicação entre cliente e servidor?
Uma mensagem de ação possui três partes:
1. O objetivo (goal)
2. O resultado (result)
3. O feedback

Cada um corresponde a um tópico e a um tipo de mensagem. Veja a baixo o diagrama de comunicação ActionClient+ActionServer;

![Diagrama de comunicação entre cliente e servidor actions](https://github.com/marcospontoexe/ROS/blob/main/imagens/action%20comunications.png)

Portanto, sempre que um servidor de ação é chamado, a sequência de etapas é a seguinte:

* Quando um cliente de ação chama um servidor de ação de um nó, o que realmente acontece é que o cliente de ação envia para o servidor de ação o **goal** (objetivo) solicitado através do tópico `/server_action_name/goal`.
* Quando o servidor de ação começa a executar o objetivo, ele envia ao cliente de ação o **feedback** através do tópico `/server_action_name/feedback`.
* Finalmente, quando o servidor de ação termina o objetivo, ele envia ao cliente de ação o **result** (resultado) através do tópico `/server_action_name/result`.

Devido à forma como as ações funcionam, na verdade, você pode chamar diretamente o servidor de ação publicando nos tópicos (emulando, assim, o que o cliente de ação em Python está fazendo).

### Axclient
Até agora, você aprendeu a enviar mensagens para um servidor de ação usando esses dois métodos:
1. Publicando diretamente no tópico `/goal` do servidor de ação.
2. Enviando a meta usando código Python.

Mas deixe-me dizer que ainda há um método que você pode usar para enviar metas a um servidor de ação, que é muito mais fácil e rápido do que os dois métodos que você aprendeu, o `axclient`.

O `axclient` é basicamente uma ferramenta de interface gráfica (GUI) fornecida pelo pacote actionlib, que permite interagir com um servidor de ação de maneira muito fácil e visual. Para iniciar o `axclient` é o comando: `rosrun actionlib_tools axclient.py /<name_of_action_server>`.
 
### O Servidor de ações
[Veja nesse pacote (my_action_server_example_pkg)](https://github.com/marcospontoexe/ROS/tree/main/Pacotes/exemplos/my_action_server_example_pkg), um Servidor de Ação que será chamado "fibonacci_as", que usará a mensagem de Ação FibonacciAction, e que terá uma função de callback chamada "goal_callback", que será ativada cada vez que um novo **goal** (objetivo) for enviado ao Servidor de Ação. Para enviar uma mensagem de Goal para o servidor, use `rostopic pub /fibonacci_as/goal actionlib_tutorials/FibonacciActionGoal [TAB][TAB]`, ou pelo GUI **axclient** `rosrun actionlib_tools axclient.py /fibonacci_as`.

[Nesse outro servidor de ações (action_server_drone_square_pkg)](https://github.com/marcospontoexe/ROS/tree/main/Pacotes/exemplos/action_server_drone_square_pkg), a mensagem **Goal** especifica o tempo em que o drone andará em linha reta, fazendo um movimento quadrado. O **feedback** deve publicar o lado atual (como um número) em que o robô está enquanto faz o quadrado. O **Result** deve publicar o número total de segundos que o drone levou para fazer o quadrado.

### Criando mensagem de ações
É sempre recomendável que você use as mensagens de ação já fornecidas pelo ROS. Estas podem ser encontradas nos seguintes pacotes do ROS:
* actionlib.
  * Test.action
  * TestRequest.action
  * TwoInts.action
* actionlib_tutorials
  * Fibonacci.action
  * Averaging.action

No entanto, pode acontecer de você precisar criar seu próprio tipo. Para criar sua própria mensagem de ação personalizada, você deve:
1. Criar um diretório chamado **action** dentro do seu pacote.
2. Criar um arquivo com a extensão **.action** dentro do diretório action. O arquivo **.action** deve conter três partes, cada parte separada por três hífens, como no exemplo a baixo:
```
#goal
package_where_message_is/message_type goal_var_name
---
#result
package_where_message_is/message_type result_var_name
---
#feedback
package_where_message_is/message_type feedback_var_name
```

Se você não precisar de uma parte da mensagem (por exemplo, não precisar fornecer feedback), então você pode deixar essa parte vazia. Mas você deve sempre especificar os separadores de hífens.

3. Modifique o arquivo **CMakeLists.txt**.
4. Modifique o arquivo **package.xml**.
5. Compilar: `catkin_make --only-pkg-with-deps nome_do_pacote` e `source devel/setup.bash`.

#### Modificando o arquivo CMakeLists.txt
Você terá que editar quatro funções dentro do CMakeLists.txt; **find_package()**, **add_action_files()**, **generate_messages()** e **catkin_package()**

1. **find_package()**: Todos os pacotes necessários para COMPILAR as mensagens de tópicos, serviços e ações devem ser listados aqui. No arquivo package.xml, você precisa declará-los como "built".
```
find_package(catkin REQUIRED COMPONENTS
  std_msgs 
  # your packages are listed here
  actionlib_msgs
)
```

2. **add_action_files()**: Esta função conterá todas as mensagens de ação deste pacote (que estão armazenadas na pasta action) que precisam ser compiladas. Coloque-as abaixo da tag FILES.
```
add_action_files(
      FILES
      Name.action
)
```

3. **generate_messages()**: Os pacotes necessários para a compilação das mensagens de ação são importados aqui. Escreva o mesmo aqui como você escreveu no find_package.
```
generate_messages(
   DEPENDENCIES
   std_msgs 
   # Your packages go here
   actionlib_msgs
 )
```

4. **catkin_package()**: Aqui estão todos os pacotes que serão necessários para alguém que execute algo do seu pacote. Todos os pacotes listados aqui devem estar no arquivo **package.xml** como `<exec_depend>`:
```
catkin_package(
      CATKIN_DEPENDS
      rospy
      # Your package dependencies go here
)
```

#### Modificando o arquivo package.xml
Adicione todos os pacotes necessários para compilar as mensagens.

1. Se, por exemplo, uma das variáveis no arquivo .action utiliza uma mensagem definida fora do pacote std_msgs, digamos "nav_msgs/Odometry", você precisará importá-la. Para fazer isso, você teria que adicionar o pacote nav_msgs como `<build_depend>`, adicionando a seguinte linha:
`<build_depend>nav_msgs</build_depend>`

2. Por outro lado, se você precisa de um pacote para a execução dos programas dentro do seu pacote, você terá que importar esses pacotes como `<exec_depend>`, adicionando a seguinte linha: **(Não esqueça de adicionar o pacote "actionlib")**
```
<build_export_depend>nav_msgs</build_export_depend>
<exec_depend>nav_msgs</exec_depend>

<build_export_depend>actionlib</build_export_depend>
<exec_depend>actionlib</exec_depend>
```

3. Ao compilar mensagens de ação personalizadas, é obrigatório adicionar actionlib_msgs como dependência de compilação (`build_dependency`): `<build_depend>actionlib_msgs</build_depend>`

4. Ao usar Python, é obrigatório adicionar rospy como dependência de execução (`run_dependency`): 
```
<build_export_depend>rospy</build_export_depend>
<exec_depend>rospy</exec_depend>
```

5. Por fim, quando tudo estiver configurado corretamente, você só precisa compilar: `catkin_make --only-pkg-with-deps nome_do_pacote` e `source devel/setup.bash`.

[Veja nesse servidor de ações (actions_quiz)](https://github.com/marcospontoexe/ROS/tree/main/Pacotes/exemplos/actions_quiz) uma mensagem chamada "CustomActionMsg.action" que pode receber **TAKEOFF** ou **LAND** como mensagem de **Goal**, para fazer o drone levantar voo atraves do tópico "/drone/takeoff", ou posar atraves do tópico "/drone/land". Como **feedback**, ele publica uma vez por segundo qual ação está acontecendo (TAKEOFF ou LAND). Quando a ação terminar, o resultado não retornará nada.

# NAVEGAÇÃO

## Criando um mapa do zero
1. Inicie o nó "**slam_gmapping**", do pacote "gmapping", com o comando `roslaunch turtlebot_navigation_gazebo gmapping_demo.launch`.
2. Em outro terminal abra o **RViz** (`rosrun rviz rviz`).
3. Adicione um LaserScan: No RViz clique em **Add** e escolha **LaserScan**, da pasta rviz, nas propriedades de exibição do Laser Scan, insira o nome do tópico onde o laser está publicando seus dados (por exemplo: **/kobuki/laser/scan**).
4. Em Global Options (Opções Globais), mude a opção **Fixed Frame** (Quadro Fixo) para **map**.
5. Para ver o robô no Rviz, você pode adicionar também um **RobotModel**, Isso mostrará a situação atual do robô na tela. Você também pode tentar exibir todos os quadros de referência do robô adicionando ao Rviz as exibições **TF** (Transform Topics).

O "mapa" do laser que é construído desaparecerá com o tempo, porque o Rviz só pode armazenar em buffer um número finito de varreduras a laser.

6. Clique no botão Add e adicione o **Map**.
7. Nas propriedades do Map, defina o tópico como **/map**.
8. **Salvando** as configurações do rviz: Vá para o canto superior esquerdo da tela do RViz e abra o menu Arquivo, selecione a opção **Save Config As**. Agora você poderá carregar sua configuração salva a qualquer momento selecionando a opção **Open Config** no menu Arquivo.

9. Abro o **TeleOp** para navegar com o robô e fazer a leitura do ambiente: `roslaunch turtlebot_teleop keyboard_teleop.launch`.

### Criando um arquivo launch para o nó slam_gmapping
Este pacote contém um wrapper ROS para o Gmapping da OpenSlam. O pacote gmapping oferece SLAM (Simultaneous Localization and Mapping) baseado em laser como um nó ROS chamado slam_gmapping. Usando o slam_gmapping, você pode criar um mapa de grade de ocupação 2D a partir de dados de laser e pose (odometria) coletados por um robô móvel.

* Subscribes to (name/type):
    * `/scan` sensor_msgs/LaserScan : Dados do laser scanner.
    * `/tf`: Odometria do robô.
      
* Publishes to (name/type):
    * `/tf`/tf/tfMessage: posição relatica no mapa.
      
* services:
      *`~dynamic_map` : retorna o mapa.

Este nó é altamente configurável e possui muitos parâmetros que podem ser alterados para melhorar o desempenho do mapeamento. Esses parâmetros serão lidos do Servidor de Parâmetros do ROS e podem ser definidos tanto no próprio arquivo launch quanto em arquivos de parâmetros separados (arquivo YAML). Se você não definir alguns parâmetros, ele usará os valores padrão. Uma lista completa dos parâmetros pode ser encontrada [nesse link](https://docs.ros.org/en/hydro/api/gmapping/html/). Vamos verificar alguns dos mais importantes:

#### General Parameters
* **base_frame** (default: "base_link"): Indica o nome do quadro ligado à base móvel.
* **map_frame** (default: "map"): Indica o nome do quadro ligado ao mapa.
* **odom_frame** (default: "odom"): Indica o nome do quadro ligado ao sistema de odometria.
* **map_update_interval** (default: 5.0): Define o tempo (em segundos) a esperar até atualizar o mapa.

#### Laser Parameters
* **maxRange** (float): Define o alcance máximo do laser. Defina este valor para algo ligeiramente superior ao alcance máximo real do sensor.
* **maxUrange** (default: 80.0): Define o alcance utilizável máximo do laser. Os feixes de laser serão cortados para este valor.
* **minimumScore** (default: 0.0): Define a pontuação mínima para considerar uma leitura do laser como boa.

#### Dimensões iniciais e resolução do mapa
* **xmin** (default: -100.0): Tamanho inicial do mapa.
* **ymin** (default: -100.0): Tamanho inicial do mapa.
* **xmax** (default: 100.0): Tamanho inicial do mapa.
* **ymax** (default: 100.0): Tamanho inicial do mapa.
* **delta** (default: 0.05): Resolução do mapa.

#### Outros parâmetros
* **linearUpdate** (default: 1.0): Define a distância linear que o robô deve se mover para processar uma leitura do laser.
* **angularUpdate** (default: 0.5): Define a distância angular que o robô deve se mover para processar uma leitura do laser.
* **temporalUpdate** (default: -1.0): Define o tempo (em segundos) de espera entre as leituras do laser. Se este valor for definido como -1,0, esta função será desativada.
* **particles** (default: 30): Número de partículas no filtro.

[Nesse pacote](https://github.com/marcospontoexe/ROS/tree/main/Pacotes/exemplos/my_mapping_launcher) chamado "my_mapping_launcher", a launch "my_gmapping_launch.launch" inicia o nó "slam_gmapping" do pacote "gmapping" passando os parâmetros iniciais para a **árvore de transformação**.

Os parâmetros podem ser alterados diretamente no arquivo de inicialização. Mas esta não é a única maneira de carregar parâmetros. Na verdade, os parâmetros geralmente são carregados de um arquivo externo. Este arquivo que contém os parâmetros geralmente é um arquivo **YAML**.

Portanto, você também pode escrever todos os parâmetros em um arquivo YAML e, em seguida, carregar esse arquivo (e os parâmetros) no arquivo de inicialização apenas adicionando a seguinte linha dentro da tag **`<node>`**: 
`<rosparam file="$(find my_mapping_launcher)/params/gmapping_params.yaml" command="load" />`.

[Nesse pacote](https://github.com/marcospontoexe/ROS/tree/main/Pacotes/exemplos/parametros_iniciais) chamado "parametros_iniciais" a launch "parametros_iniciais_launch.launch" inicia o nó "slam_gmapping" do pacote "gmapping", e inicia o arquivo "gmapping_params.yaml" que contem os parâmetros necessário para a árvore de transformação. Nesse exemplo os parâmetros são iniciados pelo arquivo .YAML, ao invés de ser iniciado pela launch.

### Salvando o map
Outro dos pacotes disponíveis no ROS Navigation Stack é o pacote **map_server**. Este pacote fornece o nó **map_saver**, que nos permite acessar os dados do mapa de um serviço ROS e salvá-los em um arquivo.

Quando você solicita ao map_saver para salvar o mapa atual, os dados do mapa são salvos em dois arquivos: um é o arquivo **YAML**, que contém os metadados do mapa e o nome da imagem, e o segundo é a própria imagem, que contém os dados codificados do mapa de grade de ocupação.

Podemos **salvar o mapa construído** a qualquer momento usando o seguinte comando: `rosrun map_server map_saver -f nome_do_mapa`. Este comando obterá os dados do mapa do tópico "map" e os gravará em 2 arquivos, **nome_do_mapa.pgm** e **nome_do_mapa.yaml**. Os arquivos serão salvos inicialmente no diretório onde você executar o comando.

#### O arquivo YAML
O arquivo YAML gerado conterá os 6 campos a seguir:
1. **Image**: Nome do arquivo que contém a imagem do Mapa gerado.
2. **Resolution**: Resolução do mapa (em metros/pixel). 
2. **origin**: Coordenadas do pixel inferior esquerdo no mapa. Essas coordenadas são fornecidas em 2D (x,y). O terceiro valor indica a rotação. Se não houver rotação, o valor será 0.
2. **occupied_thresh**: Os pixels que possuírem um valor superior a este valor serão considerados como uma zona completamente ocupada.
2. **free_thresh**: Pixels que tenham um valor menor que este valor serão considerados como uma zona completamente livre.
3. **negate**: Inverte as cores do Mapa. Por padrão, branco significa completamente livre e preto significa completamente ocupado.

#### O arquivo de imagem (PGM - Portable Gray Map) 
Para visualizar o arquivo PGM você pode fazer o seguinte:
* Abra-o através do IDE. Para poder fazer isso, o arquivo deve estar no diretório catkin_ws/src.
* Abra-o através do Web Shell. Você pode usar, por exemplo, o editor **vi** digitando o comando `vi nome_do_mapa.pgm`.
* Baixe o arquivo e visualize-o no seu computador local com o seu próprio editor de texto.

### Fornecendo o mapa
Além do nó map_saver, o pacote map_server também fornece o nó **map_server**. Este nó lê um arquivo de mapa do disco e fornece o mapa para qualquer outro nó que o solicite através de um Serviço ROS.

O serviço a ser chamado para obter o mapa é `/static_map` (nav_msgs/GetMap): Fornece os dados de ocupação do mapa através deste serviço.

Além de solicitar o mapa através do serviço "static_map", existem dois tópicos latched aos quais você pode se conectar para receber uma mensagem ROS com o mapa. Os tópicos nos quais este nó escreve os dados do mapa são:
* **map_metadata (nav_msgs/MapMetaData)**: Fornece os metadados do mapa por meio deste tópico.
* **map (nav_msgs/OccupancyGrid)**: Fornece os dados de ocupação do mapa por meio deste tópico.

**NOTA**: Quando um **tópico é latched**, significa que a última mensagem publicada nesse tópico será armazenada. Isso significa que qualquer nó que escute este tópico no futuro receberá esta última mensagem, mesmo que ninguém esteja mais publicando neste tópico. Para especificar que um tópico será latched, basta definir o atributo latch como verdadeiro ao criar o tópico.

Para lançar o nó **map_server** e fornecer informações de um mapa a partir de um arquivo de mapa, use o seguinte comando: `rosrun map_server map_server nome_do_mapa.yaml`. Se você lançar o comando a partir do diretório onde o arquivo de mapa está salvo, não precisa especificar o caminho completo para o arquivo. Caso contrário, se estiver em um diretório diferente, lembre-se de que será necessário **especificar o caminho completo para o arquivo**.

[Nesse pacote criado](https://github.com/marcospontoexe/ROS/tree/main/Pacotes/exemplos/provide_map) a launch "start_ provide_map" inicia o nó "map_server" do pacote "map_server", que fornece informações do arquivo ".yaml" para os tópicos "map_metadata", "map" e também para o serviço "static_map". Para verificar se o mapa está sendo fornecido corretamente, 
você pode usar o seguinte comando para listar os tópicos que o nó map_server está publicando: `rostopic list | grep map`. E `rosservice list | grep map` para verificar se o serviço "static_map" foi iniciado.

[Nesse cliente de serviço](https://github.com/marcospontoexe/ROS/tree/main/Pacotes/exemplos/get_map_data) criado, o nó "call_map_service.py" chama o serviço "static_map", que foi iniciado no exemplo a cima, e mostra algumas informações do mapa como, dimensão e resolução.

## Localização
Para realizar uma navegação adequada, seu robô precisa saber em qual posição do mapa ele está localizado e com qual orientação a cada momento.

O nó **amcl** obtém **dados do laser** e da **odometria** do robô, bem como do **mapa do ambiente**, e fornece uma estimativa da pose do robô. Quanto mais o robô se movimenta pelo ambiente, mais dados o sistema de localização obterá, e mais precisa será a pose estimada que ele retornará.

[No pacote](https://github.com/marcospontoexe/ROS/tree/main/Pacotes/exemplos/my_amcl_launcher) "my_amcl_launcher" a launch "change_map.launch" inicia o nó map_server, e o nó "amcl" com os parâmetros definidos na launch. Os arquivos de mapa estão localizados em um diretório chamado "maps" do pacote husky_navigation. 

[o pacote "amcl_params"](https://github.com/marcospontoexe/ROS/tree/main/Pacotes/exemplos/params_amcl) inicia o nó "map_server" e o nó "amcl", poéem dessa vez os parametros de navegação são passados para o amcl através de um arquivo **yaml** chamado "my_amcl_params".

O nó **amcl** publica a posição atual do robô no tópico **amcl_pose**, use o comando `rostopic echo -n1 /amcl_pose` para vizualizar.

[o pacote "get_position"](https://github.com/marcospontoexe/ROS/tree/main/Pacotes/exemplos/get_position) inicia o nó "service_server" através da launch "start_get_position.launch". O nó cria um servidor de serviço chamado "get_pose_service" que fica publicando a posição do robô no tópico "amcl_pose". Para vizualizar a posição publicada pelo serviço get_pose_service use o comando `rosservice call get_pose_service "{}"` en um terminal separado, e veja a posição sendo impressa no terminal do pacote get_position.

### Serviços do nó amcl 
* Fornecidos pelo nó amcl: **global_localization** (std_srvs/Empty): Inicia a localização global, onde todas as partículas são dispersas aleatoriamente por todo o espaço livre no mapa.

* Solicitados pelo nó amcl: **static_map** (nav_msgs/GetMap): amcl chama este serviço para recuperar o mapa que é usado para a localização baseada em laser, reiniciando a posição das partículas periodicamente.

[Nesse exemplo](https://github.com/marcospontoexe/ROS/tree/main/Pacotes/exemplos/initialize_particles) o cliente de serviço "init_particles_caller.py" inicio o serviço "global_localization" para disperçar as partículas, que são publicadas no tópico "particlecloud". O nó "square_move.py" faz o robô realizar um movimento em quadrado, se a covariância for menor que 0,65, significa que o robô se localizou corretamente e o programa terminará. Se a covariância for maior que 0,65, repetirá todo o processo (dispersar as partículas, realizar o movimento, verificar a covariância...). A covariancia é publicada no tópico "amcl_pose", e retorna um vetor com várias posições, os únicos valores aos quais você precisa prestar atenção são o primeiro (que é a covariância em x), o oitavo (que é a covariância em y) e o último (que é a covariância em z).

### Usando o Rviz para localização
Precisamos adicionar trê telas de vizualização **LaserScan**, **Map Display** e **PoseArray**.
1. Inicie o nó **amcl**  (Adaptive Monte Carlo Localization) para visualizar os **Pose Arrays**: `roslaunch husky_navigation amcl_demo.launch`.
2. Em outro terminal inicio o **Rviz**: `rosrun rviz rviz`.
3. Visualize **Pose Array** (Nuvens de Partículas): Clique no botão Adicionar em "displays" e escolha a exibição **PoseArray**. Nas propriedades da exibição, insira o nome do **tópico** onde a nuvem de partículas está sendo publicada (geralmente **/particlecloud**).

Para ver a posição do robô, você também pode escolher adicionar as exibições RobotModel ou TF.

4. Nas propriedades gerais mude o **Fixed Frame** para **map**.
5. Adicione um **LaserScan**: No RViz clique em Add e escolha LaserScan, da pasta rviz, nas propriedades de exibição do Laser Scan, insira o nome do tópico onde o laser está publicando seus dados (por exemplo: /scan).
6. Clique no botão Add e adicione o **Map**, nas propriedades do Map, defina o tópico como /map.
7. Salce as configurações do rviz.

### Criando um arquivo launch para o nó amcl
Este nó é altamente personalizável e podemos configurar muitos parâmetros para melhorar seu desempenho. Esses parâmetros podem ser definidos diretamente no arquivo launch ou em um arquivo de parâmetros separado (arquivo YAML). Veja nesse [link](http://wiki.ros.org/amcl) os parÂmetros configuráveis para nó amcl.
 
#### General Parameters
* **odom_model_type** (default: "diff"): Ele coloca o modelo de odometria em uso. Pode ser "diff," "omni," "diff-corrected" ou "omni-corrected."
* **odom_frame_id** (default: "odom"): Indica o frame associado à odometria.
* **base_frame_id** (default: "base_link"): Indica o frame associado à base do robô.
* **global_frame_id** (default: "map"): Indica o nome do frame de coordenadas publicado pelo sistema de localização.
* **use_map_topic** (default: false): Indica se o nó obtém os dados do mapa a partir do tópico ou de uma chamada de serviço.

#### Filter Parameters
Esses parâmetros permitem configurar a forma como o filtro de partículas opera.

* **min_particles** (default: 100): Define o número mínimo de partículas permitidas para o filtro.
* **max_particles** (default: 5000): Define o número máximo de partículas permitidas para o filtro.
* **kld_err** (default: 0.01): Define o erro máximo permitido entre a distribuição verdadeira e a distribuição estimada.
* **update_min_d** (default: 0.2): Define a distância linear (em metros) que o robô precisa mover para realizar uma atualização do filtro.
* **update_min_a** (default: π/6.0): Define a distância angular (em radianos) que o robô precisa mover para realizar uma atualização do filtro.
* **resample_interval** (default: 2): Define o número de atualizações do filtro necessárias antes da reamostragem.
* **transform_tolerance** (default: 0.1): Tempo (em segundos) com o qual a transformação publicada deve ser pós-datada, para indicar que essa transformação é válida no futuro.
* **gui_publish_rate** (default: -1.0): Taxa máxima (em Hz) na qual as varreduras e caminhos são publicados para visualização. Se esse valor for -1.0, essa função está desativada.

#### Laser Parameters
Esses parâmetros permitem configurar a forma como o nó amcl interage com o laser.

* **laser_min_range** (default: -1.0): Alcance mínimo da varredura a ser considerado; -1,0 fará com que o alcance mínimo relatado pelo laser seja usado.
* **laser_max_range** (default: -1.0): Alcance máximo da varredura a ser considerado; -1,0 fará com que o alcance máximo relatado pelo laser seja usado.
* **laser_max_beams** (default: 30): Quantos feixes uniformemente espaçados em cada varredura serão usados ao atualizar o filtro.
* **laser_z_hit** (default: 0.95): Peso da mistura para a parte z_hit do modelo.
* **laser_z_short** (default: 0.1): Peso da mistura para a parte z_short do modelo.
* **laser_z_max** (default: 0.05): Peso da mistura para a parte z_max do modelo.
* **laser_z_rand** (default: 0.05): Peso da mistura para a parte z_rand do modelo.

## Path Planning
Para uma navegação autônoma, precisaremos de algum tipo de sistema que diga ao robô ONDE ir, inicialmente, e COMO chegar lá, finalmente. No ROS, chamamos esse sistema de Planejamento de Trajetórias (Path Planning).

O path planning basicamente recebe como entrada a localização atual do robô e a posição para onde o robô deseja ir, e nos fornece como saída o caminho melhor e mais rápido para alcançar esse ponto.

Como existem muitos nós diferentes trabalhando juntos, o número de parâmetros disponíveis para configurar esses nós também é muito grande. Acho que seria uma ótima ideia se resumíssemos os diferentes arquivos de parâmetros que precisaremos configurar para o Path Planning. Os arquivos de parâmetros necessários são os seguintes:
* move_base_params.yaml
* global_planner_params.yaml
* local_planner_params.yaml
* common_costmap_params.yaml
* global_costmap_params.yaml
* local_costmap_params.yaml

### Visuzalizando um Path Planning com o Rviz 
Para ver um path plannig no rviz você precisará de três elementos **Map (Costmaps)**, **Path (Plans)** e **2D Tools**.
1. Execute o nó **move_base**: `roslaunch husky_navigation move_base_demo.launch`.
2. Em outro termial conabra o **Rviz**: `rosrun rviz rviz`.
3. Configure o **Visualize Costmaps** no **rviz**: 
    1. Clique no botão **Add** em Displays e escolha o elemento **Map**.
    2. Defina o tópico para /move_base/global_costmap/costmap para visualizar o mapa de **costmaps global**.
    3. Altere o tópico para /move_base/local_costmap/costmap para visualizar o mapa de **costmaps local**.
4. Configure o **Visualize Plans** no **rviz**: 
    1. Clique no botão **Add** em Displays e escolha o elemeto **Path**.
    2. Defina o tópico para **/move_base/NavfnROS/plan** para visualizar o **plano global**.
    3. Altere o tópico para **/move_base/DWAPlannerROS/local_plan** para visualizar o **plano local**.
5. Para que as ferramentas 2D funcionem, o **Fixed Frame** no Rviz deve estar configurado como **map**.
6. **Salve** as configurações para usar futuramente.
7. Use a ferramenta **2D Pose Estimate** para fornecer uma pose inicial para o robô.
8. Use a ferramenta **2D Nav Goal** para enviar uma pose de objetivo para o robô.

### Navigation Stack
A Navigation Stack (Pilha de Navegação) é um conjunto de nós e algoritmos ROS que são usados para mover autonomamente um robô de um ponto a outro, evitando todos os obstáculos que o robô possa encontrar em seu caminho. O ROS Navigation Stack vem com uma implementação de vários algoritmos relacionados à navegação que podem ajudá-lo a realizar navegação autônoma em seus robôs móveis.

A Navigation Stack receberá como entrada a localização atual do robô, a localização desejada para onde o robô quer ir, os dados de Odometria do Robô (codificadores de roda, IMU, GPS...) e dados de um sensor como um Laser. Em troca, ela irá produzir os comandos de velocidade necessários e enviá-los para a base móvel, a fim de mover o robô até a posição de objetivo especificada.

veja a baixo os blocos básicos de construção do Navigation Stack.

![blocos básicos de construção do Navigation Stack](https://github.com/marcospontoexe/ROS/blob/main/imagens/navstack.png)

De acordo com o diagrama mostrado, devemos fornecer alguns blocos funcionais para que funcionem e se comuniquem com a pilha de navegação. A seguir, estão explicações breves de todos os blocos que precisam ser fornecidos como entrada para a pilha de navegação do ROS:
* **Odometry source**: Os dados de odometria de um robô fornecem a posição do robô em relação à sua posição inicial. As principais fontes de odometria são os codificadores de roda, IMU e câmeras 2D/3D (odometria visual). O valor da odometria deve ser publicado para a pilha de navegação, que possui um tipo de mensagem nav_msgs/Odometry. A mensagem de odometria pode conter a posição e a velocidade do robô.
* **Sensor source**: Os sensores são utilizados em duas tarefas na navegação: uma para localizar o robô no mapa (utilizando, por exemplo, o laser) e outra para detectar obstáculos no caminho do robô (usando o laser, sonares ou nuvens de pontos).
* **sensor transforms/tf**: Os dados capturados pelos diferentes sensores do robô devem ser referenciados a um quadro de referência comum (geralmente o base_link) para que seja possível comparar dados provenientes de diferentes sensores. O robô deve publicar a relação entre o quadro de coordenadas principal do robô e os quadros dos diferentes sensores usando transformações do ROS.
* **base_controller**: A função principal do controlador de base é converter a saída da pilha de navegação, que é uma mensagem Twist (geometry_msgs/Twist), em velocidades de motor correspondentes para o robô.

### O pacote Move_Base
A função principal do nó move_base é mover o robô de sua posição atual para uma posição de objetivo. Basicamente, este nó é uma implementação de um **SimpleActionServer**, que recebe uma pose de objetivo com o tipo de mensagem geometry_msgs/PoseStamped. Portanto, podemos enviar metas de posição para este nó utilizando um SimpleActionClient.

Este **Action Server** fornece o tópico **move_base/goal**, que é a entrada da Pilha de Navegação (nav stack). Este tópico é utilizado para **fornecer a pose de objetivo**.

Veja alguns tópicos fornecidos pelo servidor de ações do move base:
* move_base/goal (move_base_msgs/MoveBaseActionGoal)
* move_base/cancel (actionlib_msgs/GoalID)
* move_base/feedback (move_base_msgs/MoveBaseActionFeedback)
* move_base/status (actionlib_msgs/GoalStatusArray)
* move_base/result (move_base_msgs/MoveBaseActionResult)

[Nesse pacote criado](https://github.com/marcospontoexe/ROS/tree/main/Pacotes/exemplos/send_goals) o cliente de ações "send_goal_client.py" envia mensagem para o nó "move_base " através do tópico **/move_base/goal** para enviar a robô até um ponto determinado.

[No pacote "move_base_parametros"](https://github.com/marcospontoexe/ROS/tree/main/Pacotes/exemplos/move_base_parametros) os **parametros do nó move_base são modificados** pelo arquivo "my_move_base_params.yaml", os parâmetros do costmap global são modificados pelo arquivo "my_global_costmap_params.yaml", os parâmetros do costmap local são modificados pelo arquivo "my_local_costmap_params.yaml", e os parâmetros do costmap comum pelo arquivo "my_common_costmap_params.yaml".

### Global Planner
Quando um novo objetivo é recebido pelo nó move_base, esse objetivo é imediatamente enviado para o Global Planner (planejador global). Em seguida, o planejador global é responsável por calcular um caminho seguro para chegar àquela **posição de objetivo**. Este caminho é calculado antes do robô começar a se mover, portanto, **não levará em consideração as leituras que os sensores do robô estão fazendo enquanto ele se move**. Cada vez que um novo caminho é planejado pelo planejador global, este caminho é publicado no tópico **/plan**.

#### Mudando o Global Planner
O Global Planner usado pelo nó **move_base** é especificado no parâmetro **base_global_planner**. Ele pode ser configurado em um arquivo de parâmetros, como no exemplo abaixo:

  ```
  base_global_planner: "navfn/NavfnROS" # Sets the Navfn Planner
  base_global_planner: "carrot_planner/CarrotPlanner" # Sets the CarrotPlanner
  base_global_planner: "global_planner/GlobalPlanner" # Sets the GlobalPlanner
  ```

Ou pode ser configurado diretamente no arquivo launch, na tag:

`<arg name="base_global_planner" default="navfn/NavfnROS"/>`

Para garantir que você alterou corretamente o Global Planner, você pode usar o seguinte comando: `rosparam get /move_base/base_global_planner`.

##### Parâmetros do Navfn 
Se você verificar o arquivo de parâmetros **.yaml**, verá os parâmetros definidos para o planejador Navfn:
```
NavfnROS:
  allow_unknown: true # Specifies whether or not to allow navfn to create plans that traverse unknown space.
  default_tolerance: 0.1 # A tolerance on the goal point for the planner.
```

Veja alguns dos parâmetros mais importantes para o planejador Navfn:
* /allow_unknown (default: true): Especifica se o Navfn deve ou não permitir a criação de planos que atravessam espaços desconhecidos.       
    * NOTA: se você estiver usando um costmap de camadas costmap_2d com uma camada de voxel ou obstáculos, você também deve definir o parâmetro track_unknown_space para essa camada como verdadeiro, caso contrário, ele converterá todo o seu espaço desconhecido em espaço livre (pelo qual o Navfn passará sem problemas).
* /planner_window_x (default: 0.0): Especifica o tamanho em x de uma janela opcional para restringir o planejador. Isso pode ser útil para restringir o NavFn a trabalhar em uma pequena janela de um costmap grande.
* /planner_window_y (default: 0.0): Especifica o tamanho em y de uma janela opcional para restringir o planejador. Isso pode ser útil para restringir o NavFn a trabalhar em uma pequena janela de um costmap grande.
* /default_tolerance (default: 0.0): Uma tolerância no ponto de destino para o planejador. O NavFn tentará criar um plano que esteja o mais próximo possível do objetivo especificado, mas não mais distante do que a tolerância padrão.
* /visualize_potential (default: false): Especifica se a área potencial calculada pelo Navfn será visualizada ou não através de um PointCloud2.

### Local Planner
Depois que o Global Planner calcula o caminho a seguir, esse caminho é enviado para o Local Planner (planejador local). O Local Planner, então, executará cada segmento do Global Planner (vamos imaginar o Local Planner como uma parte menor do Global Planner). Portanto, dado um plano a 
seguir (fornecido pelo planejador global) e um mapa, o planejador local fornecerá comandos de velocidade para mover o robô.

Ao contrário do planejador global, o planejador local **monitora a odometria e os dados do laser**, e escolhe um plano local livre de colisões para o robô. Assim, o planejador local pode recalcular o caminho do robô dinamicamente para evitar que ele colida com objetos, ao mesmo tempo permitindo que ele alcance seu destino.

Uma vez que o plano local é calculado, ele é publicado em um tópico chamado **/local_plan**. O planejador local também publica a porção do plano global que está tentando seguir no tópico **/global_plan**.

#### Mudando o Local Planner
O planejador local usado pelo nó move_base é especificado no parâmetro **base_local_planner**. Ele pode ser configurado em um arquivo de parâmetros, como no exemplo abaixo:

```
base_local_planner: "base_local_planner/TrajectoryPlannerROS" # Sets the Trajectory Rollout algorithm from base local planner
base_local_planner: "dwa_local_planner/DWAPlannerROS" # Sets the dwa local planner
base_local_planner: "eband_local_planner/EBandPlannerROS" # Sets the eband local planner
base_local_planner: "teb_local_planner/TebLocalPlannerROS" # Sets the teb local planner
```

Ou pode ser configurado diretamente no arquivo launch:

`<arg name="base_local_planner" default="dwa_local_planner/DWAPlannerROS"/>`.

Para garantir que você tenha alterado corretamente o planejador global, você pode usar o seguinte comando: `rosparam get /move_base/base_local_planner`.

Se você verificar o arquivo **my_move_base_params.yaml**, você verá os parâmetros definidos para o planejador DWAPlannerROS. A baixo os parâmetros mais importantes:
1. Robot Configuration Parameters
    * /acc_lim_x (default: 2.5): O limite de aceleração em x do robô em metros/seg^2
    * /acc_lim_th (default: 3.2): O limite de aceleração rotacional do robô em radianos/seg^2
    * /max_vel_trans (default: 0.55): O valor absoluto da velocidade translacional máxima para o robô em m/s
    * /min_vel_trans (default: 0.1): O valor absoluto da velocidade translacional mínima para o robô em m/s
    * /max_vel_x (default: 0.55): A velocidade máxima em x para o robô em m/s
    * /min_vel_x (default: 0.0): A velocidade mínima em x para o robô em m/s, negativa para movimento reverso
    * /max_vel_theta (default: 1.0): O valor absoluto da velocidade rotacional máxima para o robô em rad/s
    * /min_vel_theta (default: 0.4): O valor absoluto da velocidade rotacional mínima para o robô em rad/s
2. Goal Tolerance Parameters:
    * /yaw_goal_tolerance (double, default: 0.05): A tolerância, em radianos, para o controlador de yaw/rotação ao alcançar seu objetivo.
    * /xy_goal_tolerance (double, default: 0.10): A tolerância, em metros, para o controlador na distância x e y ao alcançar um objetivo.
    * /latch_xy_goal_tolerance (bool, default: false): Se a tolerância do objetivo é fixada (latched), se o robô atingir a localização xy do objetivo, ele simplesmente girará no lugar, mesmo que acabe fora da tolerância do objetivo enquanto faz isso.

Outros parâmetros do arquivo de configuração de parâmetros:
1. Forward Simulation Parameters:
    * /sim_time (default: 1.7): A quantidade de tempo para simular antecipadamente trajetórias em segundos. Quanto maior o parâmetro, mais longo será o plano local calculado. No entanto, também aumentará os recursos computacionais utilizados.
    * /sim_granularity (default: 0.025): O tamanho do passo, em metros, a ser utilizado entre pontos em uma trajetória dada
    * /vx_samples (default: 3): O número de amostras a serem utilizadas ao explorar o espaço de velocidade em x
    * /vy_samples (default: 10): O número de amostras a serem utilizadas ao explorar o espaço de velocidade em y
    * /vtheta_samples (default: 20): O número de amostras a serem utilizadas ao explorar o espaço de velocidade em theta
2. Trajectory Scoring Parameters:
    * /path_distance_bias (default: 32.0): O peso para quanto o controlador deve permanecer próximo ao caminho que lhe foi dado.
    * /goal_distance_bias (default: 24.0): O peso para quanto o controlador deve tentar alcançar seu objetivo local; também controla a velocidade.
    * /occdist_scale (default: 0.01):  O peso para quanto o controlador deve tentar evitar obstáculos.



### Costmaps
Um costmap é um mapa que representa os lugares onde é seguro para o robô estar em uma grade de células. Geralmente, os valores no costmap são binários, representando espaço livre ou lugares onde o robô estaria em colisão.

Cada célula em um costmap possui um valor inteiro no intervalo de {0, 255}; 
* 255 (NO_INFORMATION)
* 254 (LETHAL_OBSTACLE) 
* 253 (INSCRIBED_INFLATED_OBSTACLE)
* 0 (FREE_SPACE)

Existem dois tipos de costmaps: **costmap global** e **costmap local**. A principal diferença entre eles reside basicamente na maneira como são construídos:
* O **costmap global** é criado a partir de um mapa estático.
* O **costmap local** é criado a partir das leituras dos sensores do robô.

O Global Planner **utiliza o costmap global** para calcular o caminho a seguir.

O costmap se inscreve automaticamente nos tópicos dos sensores e se atualiza de acordo com os dados que recebe deles. Cada sensor é usado para **marcar** (inserir informações de obstáculos no costmap), **limpar** (remover informações de obstáculos do costmap) ou ambos.

Uma **operação de marcação** é apenas um índice em uma matriz para alterar o custo de uma célula.
Uma **operação de limpeza**, no entanto, consiste em traçar raios através de uma grade a partir da origem do sensor para fora para cada observação relatada. As operações de marcação e limpeza podem ser definidas na camada de obstáculos.

#### Global Costmap
O costmap global é criado a partir de um mapa estático gerado pelo usuário (como aquele que criamos no capítulo de Mapeamento). Neste caso, o costmap é inicializado para corresponder à largura, altura e informações de obstáculos fornecidas pelo mapa estático. Esta configuração é normalmente utilizada em conjunto com um sistema de localização, como o amcl. Este é o método que você utilizará para inicializar um costmap global.

O costmap global também possui seus próprios parâmetros, que são definidos em um arquivo YAML. A seguir, você pode ver um exemplo de arquivo de parâmetros para o costmap global.

```
global_frame: map
rolling_window: false

plugins:
  - {name: static, type: "costmap_2d::VoxelLayer"}
  - {name: inflation, type: "costmap_2d::InflationLayer"}
```

Veja alguns dos principais parâmetros do Global Costmap:
* global_frame (default: "map"): O quadro global no qual o costmap opera.
* robot_base_frame (default: "base_link"): O nome do quadro para o elo base do robô.
* rolling_window (default: false):  Se deve ou não usar uma versão de janela rolante do costmap.
* plugins: Sequência de especificações de plugins, uma por camada. Cada especificação é um dicionário com campos nome e tipo. O nome é usado para definir o namespace de parâmetros para o plugin. Este nome será então definido no arquivo common_costmap_parameters.yaml. O campo type  realmente define o plugin (código-fonte) que será utilizado.

Ao definir o parâmetro **rolling_window** como **false**, inicializaremos o costmap obtendo os dados de um mapa estático. Este é o método que você deseja utilizar para inicializar um costmap global.

Para configurar a **área de plugins**, adicionaremos camadas à configuração do costmap. 

Para simplificar (e esclarecer) a configuração dos costmaps, o ROS utiliza camadas. Camadas são como "blocos" de parâmetros relacionados. Por exemplo, o mapa estático, os obstáculos detectados e a inflação são separados em diferentes camadas. Essas camadas são definidas no arquivo **common_costmap_parameters.yam**l e depois adicionadas aos arquivos **local_costmap_params.yaml** e **global_costmap_params.yaml**.

Para adicionar uma camada a um arquivo de configuração de um costmap, você irá especificá-la na área de plugins:

```
plugins: 
    - {name: static_map,       type: "costmap_2d::StaticLayer"}
```

Dessa forma você está adicionando à configuração do seu costmap uma camada chamada **static_map**, que utilizará o plugin **costmap_2d::StaticLayer**. Você pode adicionar quantas camadas desejar:

```
plugins: 
    - {name: static_map,       type: "costmap_2d::StaticLayer"}
    - {name: inflation,        type: "costmap_2d::InflationLayer"}
```

Você pode ter notado que as camadas são apenas adicionadas ao arquivo de parâmetros. Isso é verdade. Tanto no arquivo de parâmetros do costmap global quanto no local, as camadas são apenas adicionadas. Os parâmetros específicos dessas camadas são definidos no arquivo de parâmetros comuns do costmap.

#### Local Costmap
A primeira coisa que você precisa saber é que o planejador local utiliza o costmap local para calcular os planos locais.

Ao contrário do costmap global, o costmap local é **criado diretamente a partir das leituras dos sensores do robô**. Dado uma largura e altura para o costmap (definidas pelo usuário), ele mantém o robô no centro do costmap enquanto se move pelo ambiente, atualizando as informações de obstáculos no mapa conforme o robô se move.

Veja alguns dos principais parâmetros do Local Costmap:
* global_frame: O quadro global no qual o costmap opera. No costmap local, este parâmetro deve ser definido como "/odom".
* robot_base_frame: O nome do quadro para o base link do robô.
* rolling_window: Se deve ou não usar uma versão de janela rolante do costmap. Se o parâmetro static_map estiver definido como true, este parâmetro deve ser definido como false. No costmap local, este parâmetro deve ser definido como "true".
* update_frequency (default: 5.0): A frequência em Hz para atualização do mapa.
* width (default: 10): A largura do costmap.
* heigth (default: 10): A altura do costmap.
* plugins: Sequência de especificações de plugins, uma por camada. Cada especificação é um dicionário com campos name e type. O "name" é usado para definir o namespace de parâmetros para o plugin.

Para o costmap local, o parâmetro **rolling_window** será definido como **true**. Desta forma, estamos indicando que não queremos que o costmap seja inicializado a partir de um mapa estático (como fizemos com o costmap global), mas que seja construído a partir das leituras dos sensores do robô. Além disso, como não teremos nenhum mapa estático, o parâmetro **global_frame** precisa ser definido como **odom**.

As camadas também podem ser adicionadas ao costmap local. No caso do costmap local, você geralmente adicionará estas 2 camadas:
* costmap_2d::ObstacleLayer: Usada para evitar obstáculos.
* costmap_2d::InflationLayer: Usada para inflar obstáculos.

Então, você terá algo assim:

```
plugins:
    - {name: obstacle_layer,      type: "costmap_2d::ObstacleLayer"}
    - {name: inflation_layer,     type: "costmap_2d::InflationLayer"}
```

**MUITO IMPORTANTE:** Note que a **camada de obstáculos** usa plugins diferentes para o **costmap local** e o **costmap global**. Para o costmap local, ela usa a **costmap_2d::ObstacleLayer**, e para o costmap global, ela usa a **costmap_2d::VoxelLayer**. Isso é muito importante porque é um erro comum na Navegação usar o plugin errado para as camadas de obstáculos.

#### Common Costmap Parameters
Esses parâmetros afetarão tanto o costmap global quanto o costmap local.
* footprint: O contorno da base móvel. No ROS, é representado por um array bidimensional na forma [x0, y0], [x1, y1], [x2, y2], ...]. Este contorno será usado para calcular o raio dos círculos inscritos e circunscritos, que são usados para inflar obstáculos de maneira que se ajustem a este robô. Normalmente, por segurança, queremos que o contorno seja ligeiramente maior do que o contorno real do robô.
* robot_radius: Caso o robô seja circular, especificaremos este parâmetro em vez do contorno.
* layers parameters: Aqui definiremos os parâmetros para cada camada. Cada camada possui seus próprios parâmetros.

Veja algumas layes:
1. Obstacle Layer: A camada de obstáculos é responsável pelas operações de marcação e limpeza. Para configurar a camada de obstáculos, primeiro precisamos definir um nome para a camada e, em seguida, configurar o parâmetro **observation_sources**.
    * observation_sources (padrão: ""): Uma lista de nomes de fontes de observação separados por espaços. Isso define cada um dos namespaces source_name definidos abaixo.
    ```
    obstacles_laser: # Name of the layer
        observation_sources: laser # We define 1 observation_source named laser
    ```
Agora podemos definir os parâmetros específicos para esta source_name. Cada source_name em observation_sources (laser para esse exemplo) define um namespace no qual os parâmetros podem ser configurados:
* /source_name/topic (padrão: source_name): O tópico no qual os dados do sensor chegam para esta fonte.
* /source_name/data_type (padrão: "PointCloud"): O tipo de dados associado ao tópico; atualmente, apenas "PointCloud", "PointCloud2" e "LaserScan" são suportados.
* /source_name/clearing (padrão: false): Se esta observação deve ser usada para limpar o espaço livre.
* /source_name/marking (padrão: true): Se esta observação deve ser usada para marcar obstáculos.
* /source_name/inf_is_valid (padrão: false): Permite valores Inf em mensagens de observação "LaserScan". Os valores Inf são convertidos para a máxima distância do laser.
* /source_name/max_obstacle_height (padrão: 2.0): A altura máxima de qualquer obstáculo a ser inserido no costmap, em metros. Este parâmetro deve ser definido ligeiramente maior que a altura do seu robô.
* /source_name/obstacle_range (padrão: 2.5): A distância máxima padrão do robô na qual um obstáculo será inserido no costmap, em metros. Isso pode ser substituído com base em cada sensor.
* /source_name/raytrace_range (padrão: 3.0): A faixa padrão em metros para traçar raios para fora do mapa usando dados do sensor. Isso pode ser substituído com base em cada sensor.

um exemplo: `laser: {data_type: LaserScan, clearing: true, marking: true, topic: scan, inf_is_valid: true, obstacle_range: 5.5}`.

A **Inflation Layer** (camada de Inflação)  é responsável por realizar a inflação em cada célula com um obstáculo:
* inflation_radius (default: 0.55): O raio em metros até o qual o mapa infla os valores de custo dos obstáculos.
* cost_scaling_factor (default: 10.0): Um fator de escala a ser aplicado aos valores de custo durante a inflação.

A **static layer** (camada estática) é responsável por fornecer o mapa estático aos costmaps que o requerem (costmap global).
* map_topic (string, padrão: "map"): O tópico ao qual o costmap se inscreve para receber o mapa estático.

### Recovery Behaviors (comportamentos de recuperação)
Pode acontecer que, ao tentar executar uma trajetória, o robô fique preso por algum motivo. Felizmente, se isso ocorrer, o ROS Navigation Stack fornece métodos que podem ajudar seu robô a se desvencilhar e continuar navegando. Esses são os comportamentos de recuperação.

O ROS Navigation Stack fornece 2 comportamentos de recuperação: **Clear Costmap** (limpar o costmap) e **Rotate Recovery** (recuperação por rotação).

Para **habilitar os comportamentos de recuperação**, precisamos definir o seguinte parâmetro no arquivo de parâmetros do **move_base**:
* recovery_behavior_enabled (padrão: true): Habilita ou desabilita os comportamentos de recuperação.

#### Rotate Recovery
Basicamente, o comportamento de recuperação por rotação é um comportamento simples que tenta liberar espaço girando o robô 360 graus. Dessa forma, o robô pode ser capaz de encontrar um caminho livre de obstáculos para continuar navegando.

Ele possui alguns parâmetros que podem ser personalizados para modificar ou melhorar seu comportamento.
1. Parâmetros de Recuperação por Rotação:
    * /sim_granularity (padrão: 0.017): A distância, em radianos, entre verificações de obstáculos ao verificar se uma rotação no lugar é segura. Padrão é 1 grau.
    * /frequency (padrão: 20.0): A frequência, em Hertz, na qual enviar comandos de velocidade para a base móvel.
2. Outros Parâmetros:
    * /yaw_goal_tolerance (double, padrão: 0.05): A tolerância, em radianos, para o controlador de orientação/rotação ao alcançar seu objetivo.
    * /acc_lim_th (double, padrão: 3.2): O limite de aceleração rotacional do robô, em radianos/segundo².
    * /max_rotational_vel (double, padrão: 1.0): A velocidade rotacional máxima permitida para a base, em radianos/segundo.
    * /min_in_place_rotational_vel (double, padrão: 0.4): A velocidade rotacional mínima permitida para a base durante rotações no lugar, em radianos/segundo.

#### Clear Costmap
A recuperação de limpeza do costmap é um comportamento simples de recuperação que limpa o espaço ao remover obstáculos fora de uma região específica do mapa do robô. Basicamente, o costmap local reverte ao mesmo estado do costmap global.

Quando você limpa obstáculos de um costmap, você torna esses **obstáculos invisíveis para o robô**. Portanto, tenha cuidado ao chamar este serviço, pois isso poderia fazer com que o robô começasse a **colidir com obstáculos**.

## Configurando o robô
No sistema de mapeamento, se não informarmos ao sistema **ONDE o robô possui o laser montado**, qual é a **orientação do laser**, qual é a **posição das rodas no robô**, etc., ele não conseguirá criar um mapa bom e preciso. 

A configuração e definição do robô são feitas nos arquivos **URDF** do robô. URDF (Unified Robot Description Format) é um formato XML que descreve o modelo de um robô. Ele define suas diferentes partes, dimensões, cinemática, dinâmica, sensores, etc...

Veja no exemplo a baixo como o laser do robô Kobuki é definido no arquivo URDF do robô:

```
<joint name="laser_sensor_joint" type="fixed">
    <origin xyz="0.0 0.0 0.435" rpy="0 0 0"/>
    <parent link="base_link"/>
    <child link="laser_sensor_link"/>
</joint>

<link name="laser_sensor_link">
        <inertial>
                <mass value="1e-5"/>
                <origin xyz="0 0 0" rpy="0 0 0"/>
                <inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6"/>
        </inertial>
        <collision>
                <origin xyz="0 0 0" rpy="0 0 0"/>
                <geometry>
                        <box size="0.1 0.1 0.1"/>
                </geometry>
        </collision>
        <visual>
                <origin xyz="0 0 0" rpy="0 0 0"/>
                <geometry>
                        <mesh filename="package://hokuyo/meshes/hokuyo.dae"/>
                </geometry>
        </visual>
</link>
```

Como você pode ver, ele define várias coisas em relação ao laser:

  * Define a posição e orientação do laser em relação à base (“base_link”) do robô.
  * Define valores relacionados à inércia.
  * Ele define valores relacionados à colisão. Esses valores fornecerão a física real do laser.
  * Ele define valores visuais. Esses valores fornecerão os elementos visuais do laser. Isso é apenas para fins de visualização.

Esses arquivos geralmente são colocados em um pacote chamado **yourrobot_description**.

### Configuração dinâmica
Até agora, vimos como alterar parâmetros modificando-os nos arquivos de parâmetros (YAML). Mas, adivinhe... essa não é a única maneira de alterar parâmetros! Você também pode modificar parâmetros dinâmicos usando a ferramenta **rqt_reconfigure**:
1. Execute o seguinte comando para abrir a ferramenta rqt_reconfigure: `rosrun rqt_reconfigure rqt_reconfigure`.
2. Abra o grupo **move_base**.
3. Selecione o grupo de parâmetros para fazer as devidas alterações de parâmetros de cada grupo.

### Transforms (transformação)
Para podermos **utilizar as leituras do laser**, precisamos definir uma transformação entre o laser e a base do robô, e adicioná-la à **árvore de transformações**. Para poder usar os dados do laser, precisamos informar ao robô ONDE (posição e orientação) este laser está montado no robô. Isso é o que chamamos de uma **transform between frames** (transformação entre quadros).

Uma transformação especifica como dados expressos em um quadro podem ser transformados em outro quadro diferente. Por exemplo, se você detectar um obstáculo com o laser a 3 cm à frente, isso significa que está a 3 cm do laser, mas não do centro do robô (geralmente chamado de **base_link**). Para saber a distância a partir do centro do robô, é necessário transformar os 3 cm do quadro **laser_frame** para o quadro **base_link**.

Quando o laser detectar um objeto, como o robô saberá onde esse objeto está? Está na frente do robô? Está atrás? Está à direita? Não há como o robô saber se não dissermos a ele a POSIÇÃO e a ORIENTAÇÃO do laser em relação ao centro do robô. Para fazer isso, precisamos seguir o seguinte:

Primeiro, vamos definir dois quadros (quadros de coordenadas), um no centro do laser e outro no centro do robô. Para navegação, é importante que o centro do robô esteja posicionado no **centro de rotação do robô**. Vamos nomear o quadro do laser como **base_laser** e o quadro do robô como **base_link**, como mostrado na figura a seguir.

![base do robô](https://github.com/marcospontoexe/ROS/blob/main/imagens/base%20link.png)

Agora, precisamos definir uma relação (em termos de posição e orientação) entre os quadros base_laser e base_link. Por exemplo, sabemos que o quadro base_laser está a uma distância de 20 cm no eixo y e 10 cm no eixo x em relação ao quadro base_link. Em seguida, precisamos fornecer esta relação ao robô. Esta relação entre a posição do laser e a base do robô é conhecida no ROS como a **TRANSFORMAÇÃO** entre o laser e o robô.

Para que o nó "slam_gmapping" funcione corretamente, você precisará fornecer 2 transformações:
* o quadro ligado ao laser -> base_link: Normalmente um valor fixo, transmitido periodicamente por um **robot_state_publisher**, ou por um **tf static_transform_publisher**.
* base_link -> odom: Normalmente fornecido pelo sistema de Odometria.

Como o robô precisa acessar essas informações a qualquer momento, iremos publicar esses dados em uma **árvore de transformações**. A árvore de transformações funciona como um banco de dados onde podemos encontrar informações sobre todas as transformações entre os diferentes quadros (elementos) do robô.

Você pode visualizar a **árvore de transformações** do seu sistema em execução a qualquer momento usando o seguinte comando: `rosrun tf2_tools view_frames.py`. Este comando irá gerar um arquivo PDF contendo um gráfico com a árvore de transformações do seu sistema.

#### Configurando a transformação entre o laser e a base do robô
Existem 2 maneiras de publicar uma transformação:
1. Usando static_transform_publisher.
2. Usando transform broadcaster.

**static_transform_publisher** é a maneira mais rápida. O static_transform_publisher é um nó pronto para uso que nos permite publicar diretamente uma transformação apenas usando a linha de comando. A estrutura do comando é a seguinte: `rosrun tf static_transform_publisher x y z yaw pitch roll frame_id child_frame_id period_in_ms`.

* x, y, z: são os offsets em metros.
* yaw, pitch, roll: são as rotações que podem ser usadas para posicionar uma transformação em qualquer orientação (valor em radianos).
* period_in_ms: especifica com que frequência enviar a transformação.

Por exemplo: `rosrun tf static_transform_publisher 0 0 0 0 0 0 map odom 100`.

Você também pode criar um arquivo launch que executa o comando acima, especificando os diferentes valores da seguinte maneira:
  ```
  <launch>
      <node pkg="tf" type="static_transform_publisher" name="name_of_node" 
            args="x y z yaw pitch roll frame_id child_frame_id period_in_ms">
      </node>
  </launch>
  ```

[Nesse pacote](https://github.com/marcospontoexe/ROS/tree/main/Pacotes/exemplos/transform_tree_kinect) chamado "transform_tree_kinect" a launch "tranform_kinect.launch" inicia o nó "static_transform_publisher". O nó publica a transformação entre a câmera Kinect montada no robô e o "base link" do robô.

A publicação das transformações também é tratada pelos arquivos **URDF**. Pelo menos, este é o uso comum. No entanto, existem alguns casos em que você precisa publicar uma transformação separadamente dos arquivos URDF. Por exemplo:
* Se você adiciona temporariamente um sensor ao robô.
* Para um sensor que não faz parte do robo, mas envia informações ao mesmo.

## Requisitos de hardware
A Navigation Stack do ROS é genérica. Isso significa que pode ser utilizada com quase qualquer tipo de robô móvel, mas existem algumas considerações de hardware que ajudarão o sistema como um todo a ter um desempenho melhor, então elas devem ser consideradas. Estes são os requisitos:
* O pacote de Navegação funcionará melhor em robôs de acionamento diferencial e robôs holonômicos. Além disso, o robô móvel deve ser controlado enviando comandos de velocidade na forma: **x, y (velocidade linear)** e **z (velocidade angular)**.
* O robô deve montar um laser planar em algum lugar ao redor do robô. Ele é usado para construir o mapa do ambiente e realizar a localização.
* Seu desempenho será melhor para bases móveis de formato quadrado e circular.

# Debugando
Usar o **Print** é a maneira mias fácil de debugar, mas também podemos usar **Logs**. Os logs permitem que você os imprima na tela, mas também os armazene no framework ROS, para que você possa classificá-los, ordená-los, filtrá-los, ou realizar outras ações.

Nos sistemas de logging, sempre há níveis de logging, como mostrado na figura a baixo. 

![níveis de log](https://github.com/marcospontoexe/ROS/blob/main/imagens/log.png)

No caso dos logs do ROS, existem cinco níveis. Cada nível inclui os níveis mais profundos. Então, por exemplo, se você usar o nível Error, todas as mensagens dos níveis Error e Fatal serão exibidas. Se o seu nível for Warning, então todas as mensagens dos níveis Warning, Error e Fatal serão exibidas.

Um bom lugar para ler todos os logs emitidos por todos os sistemas ROS é no tópico **/rosout**: `rostopic echo /rosout`.

[Veja no pacote "my_log_print_example"](https://github.com/marcospontoexe/ROS/tree/main/Pacotes/exemplos/my_log_print_example) como configurar o nível de **Log** usando a configuração `log_level=rospy.DEBUG`. Há cinco possíveis parÂmetros parra essa configuração; *DEBUG*, *INFO*, *WARN*, *ERROR* e *FATAL*.

## rqt_console
Imagine dez nós publicando dados de imagem, dados de laser, usando ações, serviços e publicando dados de depuração do seu nó de DeepLearning. É realmente difícil obter os dados de logging que você deseja.

A janela do rqt_console é dividida em três subpainéis:
1. O primeiro painel exibe os logs. Ele contém dados sobre a mensagem, severidade/nível, o nó que gerou a mensagem e outros dados. É aqui onde você extrai todos os seus dados de logs.
2. O segundo painel permite filtrar as mensagens emitidas no primeiro painel, excluindo-as com base em critérios como: nó, nível de severidade ou se contém uma determinada palavra. Para adicionar um filtro, basta clicar no sinal de adição e selecionar o desejado.
3. O terceiro painel permite destacar certas mensagens enquanto exibe as outras.

Também é importante saber que clicando na pequena engrenagem branca no canto superior direito, você pode alterar o número de mensagens mostradas. Tente manter esse número o mais baixo possível para evitar impacto de desempenho no seu sistema.

## Plot topic data e Rqt Plot
Se você precisa saber se a inclinação está correta, se a velocidade está adequada, se as leituras de torque em uma junta do braço estão acima do normal, ou se o laser está apresentando leituras anômalas. Para todos esses tipos de dados, é necessário uma ferramenta gráfica que organize de forma compreensível todos os dados que você está recebendo de forma rápida e em tempo real. É aqui que o rqt_plot se torna útil.

Digite no terminal `rqt_plot` para abrir a interface gráfica do rqt plot.

No campo de entrada de tópico localizado no canto superior esquerdo da janela, você deve digitar a estrutura do tópico que leva aos dados que deseja plotar. Lembre-se de que, para ser plotado, o tópico deve publicar um número. Após escrever o nome do tópico, pressione o SINAL DE MAIS para começar a plotar o tópico.

Para plotar as juntas do robô, precisamos plotar o tópico `/joint_states`, que possui a seguinte estrutura (que você pode obter ao extrair o tipo de mensagem do tópico com o comando `rostopic info`, seguido pelo comando `rosmsg show`):
```
std_msgs/Header header
string[] name
float64[] position
float64[] velocity
float64[] effort                                                                                                        
```

Então, para plotar a velocidade da primeira junta do robô, precisaríamos digitar `/joint_states/velocity[0]`.

Você pode adicionar quantos gráficos desejar pressionando o botão de "mais".

## Conexão entre os nós com o rqt graph 
Seu nó está conectado ao lugar correto? Por que você não está recebendo dados de um tópico? Essas perguntas são bastante comuns, como você já pode ter experimentado com sistemas ROS. O rqt_graph pode ajudá-lo a entender isso de uma maneira mais fácil. Ele exibe um gráfico visual dos nós em execução no ROS e suas conexões de tópicos. É importante destacar que ele parece ter problemas com conexões que não são tópicos.

Digite no terminal `rqt_graph` para abrir a interface gráfica do rqt graph.

Na tela do rqt graph você verá todos os nós que estão atualmente em execução, conectados pelos tópicos que utilizam para se comunicar entre si. Existem dois elementos principais que você precisa saber como usar:
1. O botão de atualização: que você deve pressionar sempre que alterar os nós que estão em execução.
2. As opções de filtro: são as três caixas ao lado do botão de atualização. O primeiro elemento permite que você selecione entre apenas nós ou tópicos. A segunda caixa permite que você filtre por nomes de nós.

## Rosbag
O rosbag registra (grava) todos os dados transmitidos pelo sistema de tópicos ROS e permite que você os reproduza a qualquer momento por meio de um arquivo simples.

* Para gravar dados dos tópicos desejados, navegue até o diretório onde salvará o arquivo e digite:
  * Para gravar nós específicos: `rosbag record -O name_bag_file.bag name_topic_to_record1 name_topic_to_record2 ... name_topic_to_recordN`
  * Para gravar todos os nós: `rosbag record -a`. Depois, você pode reproduzi-los em um sistema com apenas o roscore e obterá todos os tópicos como se tivesse o robô.


* Para extrair informações gerais sobre os dados gravados:
`rosbag info nome_arquivo_bag.bag`

* Para reproduzir os dados gravados, navegue até o diretório onde o arquivo ".bag" está salvo, e digite:
  * Para executar uma vez: `rosbag play nome_arquivo_bag.bag`
  * Para executar em loop: `rosbag play -l nome_arquivo_bag.bag`

Reproduzir os dados fará com que o rosbag publique os mesmos tópicos com os mesmos dados, no mesmo tempo em que os dados foram gravados.

Para usar arquivos rosbag, você deve garantir que o gerador de dados original (robô real ou simulação) NÃO esteja publicando. Caso contrário, você obterá dados muito estranhos (a colisão entre os dados originais e os gravados). Você também deve ter em mente que, se estiver lendo de um rosbag, o tempo é finito e cíclico, e, portanto, você deve limpar a área do gráfico para visualizar todo o período de tempo.

## RViz
RVIZ é uma ferramenta que permite visualizar Imagens, PointClouds, Lasers, Transformações Cinemáticas, Modelos de Robôs... A lista é interminável. Você pode até definir seus próprios marcadores. RVIZ é uma **representação do que está sendo publicado nos tópicos**, seja pela simulação ou pelo robô real.

**Painel Central**: Aqui é onde toda a mágica acontece. É aqui que os dados serão exibidos. É um espaço 3D que você pode rotacionar (MANTENDO O BOTÃO ESQUERDO DO MOUSE PRESSIONADO), transladar (MANTENDO O BOTÃO CENTRAL DO MOUSE PRESSIONADO) e dar zoom in/out (MANTENDO O BOTÃO DIREITO DO MOUSE PRESSIONADO).

**Painel de Exibição à Esquerda**: Aqui você gerencia/configura todos os elementos que deseja visualizar no painel central. Você só precisa usar dois elementos:
* Em Opções Globais, você deve selecionar o Quadro Fixo (Fixed Frame) que se adequa à visualização dos dados. É o quadro de referência a partir do qual todos os dados serão referenciados.
* O botão Adicionar (Add). Clicando aqui, você obtém todos os tipos de elementos que podem ser representados no RVIZ.

# Instalando o ROS Noetic
1. Primeiramente, você precisará **configurar seu computador para poder baixar pacotes** do **packages.ros.org**. Para isso, execute o comando no seu terminal local: `sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'`.

2. Em seguida, você irá **baixar a chave do servidor de chaves** usando o seguinte comando: `wget http://packages.ros.org/ros.key -O - | sudo apt-key add -`

3. Agora estamos prontos para instalar o ROS de fato. Primeiro, vamos garantir que nosso índice de **pacotes Debian esteja atualizado**. Para isso, execute o seguinte comando: `sudo apt-get update`

4. Agora você está pronto para começar a **instalar pacotes do ROS no seu sistema**. Para ter todos os pacotes básicos para começar a trabalhar com o ROS, recomendamos que você instale a instalação Desktop Full. Para isso, você pode executar o seguinte comando: `sudo apt-get install ros-noetic-desktop-full`

5. Eventualmente, você precisará **instalar alguns pacotes adicionais**. Para instalar um pacote específico do ROS, você só precisa usar a seguinte estrutura de comando: `sudo apt-get install ros-noetic-<PACKAGE_NAME>`. **Por exemplo** `sudo apt-get install ros-noetic-slam-gmapping`

## Rosdep
Antes de começar a usar o ROS, no entanto, você precisará inicializar o rosdep. O rosdep permitirá que você instale facilmente dependências do sistema e também é necessário para executar alguns componentes principais do ROS. Para inicializar o rosdep, execute o seguinte comando: 
1. `sudo rosdep init`
2. `rosdep update`

## Configuração do ambiente
Finalmente, também é recomendável adicionar automaticamente as Variáveis de Ambiente do ROS à sua sessão bash toda vez que uma nova shell for iniciada. Para isso, você pode executar o seguinte comando: `source /opt/ros/noetic/setup.bash`

Você precisará executar este comando em cada nova shell que abrir para ter acesso aos comandos do ROS, a menos que adicione esta linha ao seu arquivo .bashrc. Portanto, a menos que deseje executar esse comando toda vez que abrir uma nova shell, você deve adicioná-lo ao seu **.bashrc**. Para isso, você pode executar o seguinte comando: `echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc`

Com o comando acima, você adicionará a linha `source /opt/ros/noetic/setup.bash` ao seu arquivo **.bashrc**. Dessa forma, cada vez que você abrir uma nova shell no seu computador, todas as variáveis de ambiente do ROS serão configuradas automaticamente.

Além disso, esse processo permite que você instale várias distribuições do ROS (por exemplo, Indigo e Noetic) no mesmo computador e alterne entre elas. Então, por exemplo, se você também tivesse o ROS Indigo instalado no seu computador local, poderia alternar entre as duas distribuições usando os comandos abaixo: 
1. Para usar o Indigo: `source /opt/ros/indigo/setup.bash`
2. Para usar o Noetic: `source /opt/ros/noetic/setup.bash`

## Dependências para a construção de pacotes
Existem várias ferramentas que você também precisará para **gerenciar seus espaços de trabalho do ROS**. Para instalar todas essas ferramentas, você pode executar o seguinte comando: `sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential`

## Teste sua configuração
**Roscore** é o processo principal que gerencia todos os sistemas do ROS. Portanto, se quisermos fazer qualquer coisa com o ROS, sempre precisamos primeiro iniciar o roscore um terminal, Execute o comando `roscore`.

Você pode executar o comando `roscd` para garantir que seu sistema ROS esteja configurado corretamente. Se tudo correr bem, você deverá ir para o seguinte caminho: `/opt/ros/kinetic$`.

## Como gerenciar espaços de trabalho do ROS no meu computador local
Primeiro, vamos criar a pasta onde criaremos nosso espaço de trabalho do ROS: 
1. Criando a pasta: `mkdir -p ~/mynew_ws/src`
2. ENtrando na pasta: `cd ~/mynew_ws/`
3. Compilando a pasta: `catkin_make`

Neste exemplo o espaço de trabalho criado se chama "mynew_ws", mas poderia ser chamado, popularmente, de catkin_ws (catkin workspace). Ao compilar o seu espaço de trabalho, ele criará o arquivo **CMakeLists.txt** na sua pasta src. Além disso, se você verificar no diretório atual, agora deverá ter uma pasta **build** e uma pasta **devel**. Dentro da pasta devel, você pode ver que agora existem vários arquivos setup.*sh. 

4. **Sourcing** qualquer um desses arquivos irá sobrescrever este espaço de trabalho sobre o seu ambiente: `source devel/setup.bash`

Para garantir que seu espaço de trabalho esteja corretamente sobreposto pelo script de configuração, você pode **verificar a variável de ambiente ROS_PACKAGE_PATH** com o seguinte comando: `echo $ROS_PACKAGE_PATH`. Se tudo ocorrer bem deverá ser retornado:
```
user:~/mynew_ws$ echo $ROS_PACKAGE_PATH
/home/user/mynew_ws/src:/home/user/catkin_ws/src:/home/simulations/public_sim_ws/src:/opt/ros/noetic/share
```



