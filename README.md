# ROS
É um framework de código aberto utilizado para o desenvolvimento de software para robôs. Embora seja chamado de "sistema operacional", o ROS não é um sistema operacional no sentido tradicional, mas sim um conjunto de ferramentas, bibliotecas e convenções que visam simplificar o desenvolvimento de software para robótica. O ROS fornece funcionalidades como gerenciamento de dispositivos, controle de hardware, troca de mensagens entre processos e visualização de dados, facilitando a criação de sistemas complexos de robótica. Ele é amplamente utilizado na comunidade de pesquisa e desenvolvimento de robótica devido à sua flexibilidade e modularidade.

## Tópicos
O ROS lida quase que inteiramente com suas comunicações por meio de tópicos. Até mesmo sistemas de comunicação mais complexos, como **serviços** ou **ações**, dependem, em última análise, de tópicos. É por isso que eles são tão importantes! Através dos tópicos do ROS, você será capaz, por exemplo, de se comunicar com seu robô para fazê-lo se mover, ler as leituras dos sensores do seu robô e muito mais.

### Publishers


## Serviços
Os serviços permitem que você desenvolva uma funcionalidade específica para seu robô e depois a disponibilize para que qualquer pessoa possa chamá-la. Por exemplo, você poderia criar um serviço que faça seu robô se mover por um período específico de tempo e depois parar.

Os serviços são um pouco mais complexos do que os tópicos, pois são estruturados em duas partes. De um lado, você tem o **Servidor de Serviço**, que fornece a funcionalidade para qualquer pessoa que queira usá-la (chamá-la). Do outro lado, você tem o **Cliente de Serviço**, que é aquele que faz a chamada/solicitação da funcionalidade do serviço.

O serviço deve estar em funcionamento antes que você possa chamá-lo. Portanto, certifique-se de ter iniciado o serviço antes de chamá-lo.

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
5. Isso criará dentro do nosso diretório src um novo pacote com alguns arquivos e diretórios (launch, src, CMakeLists.txt e package.xml).

Para verificar se nosso pacote foi criado com sucesso, podemos usar alguns comandos ROS relacionados a pacotes. Por exemplo, vamos digitar:
* `rospack list | grep nome_do_pacote`: Para filtrar, de todos os pacotes localizados no sistema ROS, o pacote chamado "nome_do_pacote".
* `roscd nome_do_pacote`: Leva você à localização no disco rígido do pacote chamado "nome_do_pacote".

Dentro do pacote deve conter;
* Um diretório chamado **src**: dentro desse diretório deve ficar o arquivo python. Verifique se o arquivo tem permissão de execusão.
  * A primeira linha do arquivo deve conter **#! /usr/bin/env python **.
  * Deve importar a bilioteca **rospy** (import rospy).
* Um diretório chamado **lounch**: Dentro desse diretório deve conter um arquivo de extensão **.launch**. O arquivo louch deve conter algo semelhante com o que foi descrito no tópico a cima; "Como a launch file funciona?".

Para executar o programa criado no pacote, execute o comando `roslaunch nome_do_pacote nome_package_launch_file.launch`.

Às vezes, o ROS não detectará um novo pacote quando você acabou de criá-lo, então você não poderá fazer um `roslaunch`. Nesse caso, você pode forçar o ROS a atualizar sua lista de pacotes com o comando: `rospack profile`.

[Veja nesse exemplo](https://github.com/marcospontoexe/ROS/tree/main/Pacotes/exemplos/catkin_ws/src) um pacote criado.

## Os nós do ROS
Nós do ROS são basicamente programas feitos no ROS. O comando ROS para ver quais nós estão realmente em execução em um computador é: `rosnode list`.

Para ver informações sobre um nó, podemos usar o comando: `rosnode info nome_do_nó`.

## Compilando um pacote
Quando você cria um pacote, geralmente precisará compilá-lo para fazê-lo funcionar. Existem diferentes métodos que podem ser usados para compilar seus pacotes ROS, o mais comum: **catkin_make**.

Este comando irá compilar todo o seu diretório src, e ele precisa ser executado no seu diretório catkin_ws para funcionar (`cd ~/catkin_ws`). Se você tentar compilar a partir de outro diretório, não funcionará.

Depois de compilar, também é muito importante "sourcer" (fornecer) o seu espaço de trabalho. Isso garantirá que o ROS sempre obtenha as últimas alterações feitas no seu espaço de trabalho: `source devel/setup.bash`.

Às vezes (por exemplo, em projetos grandes), você não vai querer compilar todos os seus pacotes, mas apenas aquele(s) onde você fez alterações. Você pode fazer isso com o seguinte comando: `catkin_make --only-pkg-with-deps <nome_do_pacote>`.

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
