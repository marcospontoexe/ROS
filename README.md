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
Adicione todos os pacotes necessários para compilar as mensagens. Neste caso, você só precisa adicionar o **message_generation**. Você terá que importar esses pacotes como <build_depend>. Por outro lado, se você precisa de um pacote para a execução dos programas dentro do seu pacote, você terá que importar esses pacotes como <exec_depend>.

Neste caso, você só precisará adicionar estas 3 linhas ao seu arquivo package.xml:
    ```
    <build_depend>message_generation</build_depend>
    <build_export_depend>message_runtime</build_export_depend>
    <exec_depend>message_runtime</exec_depend>
    ```

Depois de terminar, compile (catkin_make) seu pacote e faça o sourcement (source devel/setup.bash) das mensagens recém-geradas.

Para verificar se você tem o novo serviço de mensagem no seu sistema, pronto para ser utilizado, digite o comando `rossrv list | grep MyCustomServiceMessage`.

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

# NAVEGAÇÃO

## Criando um mapa global
1. Inicie o nó "**slam_gmapping**", do pacote "gmapping", com o comando `roslaunch turtlebot_navigation_gazebo gmapping_demo.launch`.
2. Em outro terminal use o **teleOP** para navegar pelo ambiente: `roslaunch turtlebot_teleop keyboard_teleop.launch`.
3. Em outro terminal abra o **RViz** (`roslaunch turtlebot_rviz_launchers view_mapping.launch`) para ver o mapa sendo criado.

### Criando um mapa do zero
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

Para lançar o nó **map_server** e fornecer informações de um mapa a partir de um arquivo de mapa, use o seguinte comando: `rosrun map_server map_server nome_do_mapa.yaml`. Se você lançar o comando a partir do diretório onde o arquivo de mapa está salvo, não precisa especificar o caminho completo para o arquivo. Caso contrário, se estiver em um diretório diferente, lembre-se de que será necessário especificar o caminho completo para o arquivo.

[Nesse pacote criado](https://github.com/marcospontoexe/ROS/tree/main/Pacotes/exemplos/provide_map) a launch "start_ provide_map" inicia o nó "map_server" do pacote "map_server", que fornece informações do arquivo ".yaml" para os tópicos "map_metadata", "map" e também para o serviço "static_map". Para verificar se o mapa está sendo fornecido corretamente, 
você pode usar o seguinte comando para listar os tópicos que o nó map_server está publicando: `rostopic list | grep map`. E `rosservice list | grep map` para verificar se o serviço "static_map" foi iniciado.

[Nesse cliente de serviço](https://github.com/marcospontoexe/ROS/tree/main/Pacotes/exemplos/get_map_data) criado, o nó "call_map_service.py" chama o serviço "static_map", que foi iniciado no exemplo a cima, e mostra algumas informações do mapa como, dimensão e resolução.

## Localização
Para realizar uma navegação adequada, seu robô precisa saber em qual posição do mapa ele está localizado e com qual orientação a cada momento. 

1. Execute o comando `roslaunch turtlebot_navigation_gazebo amcl_demo.launch` para iniciar a demonstração de Localização.
2. Execute o TeleOP em outro terminal para navegar pelo ambiente `roslaunch turtlebot_teleop keyboard_teleop.launch`.
3. Inicie o Rviz em outro terminal para ver a localização do robo em tempo real: `roslaunch turtlebot_rviz_launchers view_localization.launch`.

[No pacote]() "my_amcl_launcher" a launch "change_map.launch" inicia o nó map_server com os parâmetros definidos na launch. Os arquivos de mapa estão localizados em um diretório chamado "maps" do pacote husky_navigation. 

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

## Path Planning
Para uma navegação autônoma, precisaremos de algum tipo de sistema que diga ao robô ONDE ir, inicialmente, e COMO chegar lá, finalmente. No ROS, chamamos esse sistema de Planejamento de Trajetórias (Path Planning).

O path planning basicamente recebe como entrada a localização atual do robô e a posição para onde o robô deseja ir, e nos fornece como saída o caminho melhor e mais rápido para alcançar esse ponto.

1. Execute `roslaunch turtlebot_navigation_gazebo move_base_demo.launch` para iniciar um Path Planning.
2. Execute em outro terminal o RViz `roslaunch turtlebot_rviz_launchers view_planning.launch`.
3. Uso o botão "2D Nav Goal" do RViz para indicar o ponto de chegada do robô.

## Configurando o robô
No sistema de mapeamento, se não informarmos ao sistema ONDE o robô possui o laser montado, qual é a orientação do laser, qual é a posição das rodas no robô, etc., ele não conseguirá criar um mapa bom e preciso. 

A configuração e definição do robô são feitas nos arquivos URDF do robô. URDF (Unified Robot Description Format) é um formato XML que descreve o modelo de um robô. Ele define suas diferentes partes, dimensões, cinemática, dinâmica, sensores, etc...

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

### Transforms (transformação)
Para podermos utilizar as leituras do laser, precisamos definir uma transformação entre o laser e a base do robô, e adicioná-la à árvore de transformações. Para poder usar os dados do laser, precisamos informar ao robô ONDE (posição e orientação) este laser está montado no robô. Isso é o que chamamos de uma **transform between frames** (transformação entre quadros).

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

#### Criando um arquivo launch para o nó slam_gmapping
Este nó é altamente configurável e possui muitos parâmetros que podem ser alterados para melhorar o desempenho do mapeamento. Esses parâmetros serão lidos do Servidor de Parâmetros do ROS e podem ser definidos tanto no próprio arquivo launch quanto em arquivos de parâmetros separados (arquivo YAML). Se você não definir alguns parâmetros, ele usará os valores padrão. Vamos verificar alguns dos mais importantes:

##### GENERAL PARAMETERS
* **base_frame** (default: "base_link"): Indica o nome do quadro ligado à base móvel.
* **map_frame** (default: "map"): Indica o nome do quadro ligado ao mapa.
* **odom_frame** (default: "odom"): Indica o nome do quadro ligado ao sistema de odometria.
* **map_update_interval** (default: 5.0): Define o tempo (em segundos) a esperar até atualizar o mapa.

##### LASER PARAMETERS
* **maxRange** (float): Define o alcance máximo do laser. Defina este valor para algo ligeiramente superior ao alcance máximo real do sensor.
* **maxUrange** (default: 80.0): Define o alcance utilizável máximo do laser. Os feixes de laser serão cortados para este valor.
* **minimumScore** (default: 0.0): Define a pontuação mínima para considerar uma leitura do laser como boa.

##### DIMENSÕES INICIAIS E RESOLUÇÃO DO MAPA
* **xmin** (default: -100.0): Tamanho inicial do mapa.
* **ymin** (default: -100.0): Tamanho inicial do mapa.
* **xmax** (default: 100.0): Tamanho inicial do mapa.
* **ymax** (default: 100.0): Tamanho inicial do mapa.
* **delta** (default: 0.05): Resolução do mapa.

##### OUTROS PARÂMETROS
* **linearUpdate** (default: 1.0): Define a distância linear que o robô deve se mover para processar uma leitura do laser.
* **angularUpdate** (default: 0.5): Define a distância angular que o robô deve se mover para processar uma leitura do laser.
* **temporalUpdate** (default: -1.0): Define o tempo (em segundos) de espera entre as leituras do laser. Se este valor for definido como -1,0, esta função será desativada.
* **particles** (default: 30): Número de partículas no filtro.

[Nesse pacote](https://github.com/marcospontoexe/ROS/tree/main/Pacotes/exemplos/my_mapping_launcher) chamado "my_mapping_launcher", a launch "my_gmapping_launch.launch" inicia o nó "slam_gmapping" do pacote "gmapping" passando os parâmetros iniciais para a **árvore de transformação**.

Os parâmetros podem ser alterados diretamente no arquivo de inicialização. Mas esta não é a única maneira de carregar parâmetros. Na verdade, os parâmetros geralmente são carregados de um arquivo externo. Este arquivo que contém os parâmetros geralmente é um arquivo **YAML**.

Portanto, você também pode escrever todos os parâmetros em um arquivo YAML e, em seguida, carregar esse arquivo (e os parâmetros) no arquivo de inicialização apenas adicionando a seguinte linha dentro da tag **`<node>`**: 
`<rosparam file="$(find my_mapping_launcher)/params/gmapping_params.yaml" command="load" />`.

[Nesse pacote](https://github.com/marcospontoexe/ROS/tree/main/Pacotes/exemplos/parametros_iniciais) chamado "parametros_iniciais" a launch "parametros_iniciais_launch.launch" inicia o nó "slam_gmapping" do pacote "gmapping", e inicia o arquivo "gmapping_params.yaml" que contem os parâmetros necessário para a árvore de transformação. Nesse exemplo os parâmetros são iniciados pelo arquivo .YAML, ao invés de ser iniciado pela launch.

## Navigation Stack
A Navigation Stack (Pilha de Navegação) é um conjunto de nós e algoritmos ROS que são usados para mover autonomamente um robô de um ponto a outro, evitando todos os obstáculos que o robô possa encontrar em seu caminho. O ROS Navigation Stack vem com uma implementação de vários algoritmos relacionados à navegação que podem ajudá-lo a realizar navegação autônoma em seus robôs móveis.

A Navigation Stack receberá como entrada a localização atual do robô, a localização desejada para onde o robô quer ir, os dados de Odometria do Robô (codificadores de roda, IMU, GPS...) e dados de um sensor como um Laser. Em troca, ela irá produzir os comandos de velocidade necessários e enviá-los para a base móvel, a fim de mover o robô até a posição de objetivo especificada.

veja a baixo os blocos básicos de construção do Navigation Stack.

![blocos básicos de construção do Navigation Stack](https://github.com/marcospontoexe/ROS/blob/main/imagens/navstack.png)

De acordo com o diagrama mostrado, devemos fornecer alguns blocos funcionais para que funcionem e se comuniquem com a pilha de navegação. A seguir, estão explicações breves de todos os blocos que precisam ser fornecidos como entrada para a pilha de navegação do ROS:
* **Odometry source**: Os dados de odometria de um robô fornecem a posição do robô em relação à sua posição inicial. As principais fontes de odometria são os codificadores de roda, IMU e câmeras 2D/3D (odometria visual). O valor da odometria deve ser publicado para a pilha de navegação, que possui um tipo de mensagem nav_msgs/Odometry. A mensagem de odometria pode conter a posição e a velocidade do robô.
* **Sensor source**: Os sensores são utilizados em duas tarefas na navegação: uma para localizar o robô no mapa (utilizando, por exemplo, o laser) e outra para detectar obstáculos no caminho do robô (usando o laser, sonares ou nuvens de pontos).
* **sensor transforms/tf**: Os dados capturados pelos diferentes sensores do robô devem ser referenciados a um quadro de referência comum (geralmente o base_link) para que seja possível comparar dados provenientes de diferentes sensores. O robô deve publicar a relação entre o quadro de coordenadas principal do robô e os quadros dos diferentes sensores usando transformações do ROS.
* **base_controller**: A função principal do controlador de base é converter a saída da pilha de navegação, que é uma mensagem Twist (geometry_msgs/Twist), em velocidades de motor correspondentes para o robô.

## Requisitos de hardware
A Navigation Stack do ROS é genérica. Isso significa que pode ser utilizada com quase qualquer tipo de robô móvel, mas existem algumas considerações de hardware que ajudarão o sistema como um todo a ter um desempenho melhor, então elas devem ser consideradas. Estes são os requisitos:
* O pacote de Navegação funcionará melhor em robôs de acionamento diferencial e robôs holonômicos. Além disso, o robô móvel deve ser controlado enviando comandos de velocidade na forma: **x, y (velocidade linear)** e **z (velocidade angular)**.
* O robô deve montar um laser planar em algum lugar ao redor do robô. Ele é usado para construir o mapa do ambiente e realizar a localização.
* Seu desempenho será melhor para bases móveis de formato quadrado e circular.

