# ROS
É um framework de código aberto utilizado para o desenvolvimento de software para robôs. Embora seja chamado de "sistema operacional", o ROS não é um sistema operacional no sentido tradicional, mas sim um conjunto de ferramentas, bibliotecas e convenções que visam simplificar o desenvolvimento de software para robótica. O ROS fornece funcionalidades como gerenciamento de dispositivos, controle de hardware, troca de mensagens entre processos e visualização de dados, facilitando a criação de sistemas complexos de robótica. Ele é amplamente utilizado na comunidade de pesquisa e desenvolvimento de robótica devido à sua flexibilidade e modularidade.

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

![tipo de mensagens](https://github.com/marcospontoexe/ROS/blob/main/imagens/tipo%20de%20mensagens.png).

As mensagens são definidas em arquivos **.msg**, que estão localizados dentro de um diretório msg de um pacote.

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
Dentro do diretório "msg" crie um arquivo com extensão **.msg**, e coloque as variáveis usadas nesse arquivo, para o nosso exemplo veja a baixo:
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
2. `catkin_make`
3. `source devel/setup.bash`, Isso executa este arquivo bash que configura, entre outras coisas, as novas mensagens geradas criadas através do catkin_make. Se você não fizer isso, pode ocorrer um erro de importação em Python, dizendo que não encontra a mensagem gerada.

Para verificar se sua mensagem foi criada com sucesso, digite em seu terminal `rosmsg show nome_da_mensagem_criada` (para esse exemplo `rosmsg show Age`). Se a estrutura da mensagem aparecer, significa que sua mensagem foi criada com sucesso e está pronta para ser usada em seus programas ROS.

[Veja nesse nó](https://github.com/marcospontoexe/ROS/tree/main/Pacotes/exemplos/my_subscriber_odometry_pkg) um mensagem do tipo float32 criada para indicar a idade, com anos, meses e dias.

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

O serviço deve estar em funcionamento antes que você possa chamá-lo. Portanto, certifique-se de ter iniciado o serviço antes de chamá-lo. 

Para visualizar a lista de serviços ativos digite o comando `rosservice list`.

Para ter informações sobre um serviço digite `rosservice info /name_of_your_service`. Este comando retorna quatro dados:
* Node: Indica o nó que fornece (criou) esse serviço.
* URI: O URI onde o serviço pode ser acessado.
* Type: Refere-se ao tipo de mensagem usada por este serviço. Tem a mesma estrutura que os tópicos. É sempre composto por pacote_onde_a_mensagem_do_serviço_é_definida / Nome_do_Arquivo_onde_a_mensagem_do_Serviço_é_definida.
* Args: Aqui você pode encontrar os argumentos que este serviço aceita quando é chamado (valor, path, comando). 

Você pode chamar um serviço manualmente a partir do terminal. Isso é muito útil para testes e para ter uma ideia básica de como o serviço funciona: `rosservice call /the_service_name TAB+TAB`. Quando você pressiona [TAB]+[TAB] rapidamente, um elemento extra aparece (**"traj_name: ''"**), coloque o nome da função desejada a ser executada pelo serviço dentro das aspas simples.

[Veja nesse exemplo](https://github.com/marcospontoexe/ROS/tree/main/Pacotes/exemplos/my_service_client_example_pkg) como enviar um Request de um serviço do tipo cliente através de uma mensagem de serviço. Para enviar a mensagem de serviço, o serviço deve estar em execução. Execute o serviço com o comando: `roslaunch trajectory_by_name start_service.launch`.

[Esse exemplo](https://github.com/marcospontoexe/ROS/tree/main/Pacotes/exemplos/move_arm) inicia o serviço "execute_trajectory", que é inicializado pela launch "start_service", que está no pacote "iri_wam_reproduce_trajectory". Ao iniciar o serviço cliente "execute_trajectory", o arquivo python realiza uma conexão com "execute_trajectory" e inicia uma mensagem de serviço do tipo "ExecTraj", do pacote "iri_wam_reproduce_trajectory", para fazer o braço do robô se mover seguindo uma trajetória especificada em um arquivo, que é solicitado pelo Request do serviço "execute_trajectory".

[Veja nesse exemplo](https://github.com/marcospontoexe/ROS/tree/main/Pacotes/exemplos/my_service_server_example_pkg) um servidor de serviços chamado "my_service" que recebe uma mensagem de servido *Request* do tipo "Empty", e envia uma mensagem de serviço *Response* do tipo EmptyResponse.

[Nesse exemplo](https://github.com/marcospontoexe/ROS/tree/main/Pacotes/exemplos/move_around_server_pkg) a launch "start_bb8_move_in_circle_service_server" inicia um servidor de serviço chamado "move_bb8_in_circle", que aceita uma mensagem de serviço vazia e ativa o movimento circular do robo, através da mensagem "Twist" enviada para o tópico "cmd_vel". A launch "call_bb8_move_in_circle_service_server" inicia um cliente de serviço que chama o serviço "move_bb8_in_circle" iniciado pela launch "start_bb8_move_in_circle_service_server". Para o cliente de serviço rodar é necessário ter iniado o servidor de serviço.


### Mensagens de um serviço
Arquivos de mensagem de serviço têm a extensão **.srv** e são definidos dentro de um diretório chamado **srv**.

Para explorar a estrutura de uma mensagem de serviço use o comando comando `rossrv show name_of_the_package/Name_of_Service_message`. Name_of_Service_message é o Nome_do_Arquivo_onde_a_mensagem_do_Serviço_é_definida, mostrado pelo comando (`rosservice info /name_of_your_service`). Na imagens a baixo é mostrado o resutado dos dois comandos.

![estrutura de uma mensagem de serviço](https://github.com/marcospontoexe/ROS/blob/main/imagens/mensagem%20de%20um%20servi%C3%A7o.png)

As mensagens de um serviço tem duas partes **Request** e **Response**. Na figura a cima, request contém uma string chamada "traj_name" e response é composta por um booleano chamado "success" e uma string chamada "status_message".

Request é a parte da mensagem que significa quais variáveis você terá que passar para o Servidor de Serviço para que ele seja capaz de concluir sua tarefa.

Response é a parte da mensagem de serviço que define como o seu serviço responderá após concluir sua funcionalidade. Se, por exemplo, ele retornará uma string com uma mensagem específica dizendo que tudo correu bem, ou se não retornará nada, etc...

Sempre que uma mensagem de serviço é compilada, é retornado três objetos:
1. A própria mensagem de serviço: É usado para criar uma conexão com o servidor de serviço, como demonstrado no exemplo anterior: ``` traj_by_name_service = rospy.ServiceProxy('/trajectory_by_name', TrajByName)```.
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

#### Editando o arquivo package.xml.
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
1. Adicione todos os pacotes necessários para compilar as mensagens. Neste caso, você só precisa adicionar o message_generation. Você terá que importar esses pacotes como <build_depend>.


## Ações
O ROS também fornece ações. As ações são semelhantes aos serviços, no sentido de que também permitem que você codifique uma funcionalidade para o seu robô e, em seguida, a disponibilize para que qualquer pessoa possa chamá-la. A principal diferença entre ações e serviços é que, ao chamar um serviço, o robô precisa esperar até que o serviço tenha terminado antes de fazer algo mais. Por outro lado, ao chamar uma ação, o seu robô ainda pode continuar fazendo outra coisa enquanto executa a ação.

Há outras diferenças, como uma ação permitindo que você forneça feedback enquanto a ação está sendo realizada.

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
4. Agora estamos prontos para criar nosso primeiro pacote! Para criar um pacote, digite no seu terminal: `catkin_create_pkg nome_do_pacote package_dependencies`. O **nome_do_pacote** é o nome do pacote que você deseja criar, e o **package_dependencies** são os nomes de outros pacotes ROS dos quais seu pacote depende.
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

