# ROS
É um framework de código aberto utilizado para o desenvolvimento de software para robôs. Embora seja chamado de "sistema operacional", o ROS não é um sistema operacional no sentido tradicional, mas sim um conjunto de ferramentas, bibliotecas e convenções que visam simplificar o desenvolvimento de software para robótica. O ROS fornece funcionalidades como gerenciamento de dispositivos, controle de hardware, troca de mensagens entre processos e visualização de dados, facilitando a criação de sistemas complexos de robótica. Ele é amplamente utilizado na comunidade de pesquisa e desenvolvimento de robótica devido à sua flexibilidade e modularidade.

## Tópicos
O ROS lida quase que inteiramente com suas comunicações por meio de tópicos. Até mesmo sistemas de comunicação mais complexos, como **serviços** ou **ações**, dependem, em última análise, de tópicos. É por isso que eles são tão importantes! Através dos tópicos do ROS, você será capaz, por exemplo, de se comunicar com seu robô para fazê-lo se mover, ler as leituras dos sensores do seu robô e muito mais.

## Serviços
Os serviços permitem que você desenvolva uma funcionalidade específica para seu robô e depois a disponibilize para que qualquer pessoa possa chamá-la. Por exemplo, você poderia criar um serviço que faça seu robô se mover por um período específico de tempo e depois parar.

Os serviços são um pouco mais complexos do que os tópicos, pois são estruturados em duas partes. De um lado, você tem o **Servidor de Serviço**, que fornece a funcionalidade para qualquer pessoa que queira usá-la (chamá-la). Do outro lado, você tem o **Cliente de Serviço**, que é aquele que faz a chamada/solicitação da funcionalidade do serviço.

O serviço deve estar em funcionamento antes que você possa chamá-lo. Portanto, certifique-se de ter iniciado o serviço antes de chamá-lo.

## Ações
O ROS também fornece ações. As ações são semelhantes aos serviços, no sentido de que também permitem que você codifique uma funcionalidade para o seu robô e, em seguida, a disponibilize para que qualquer pessoa possa chamá-la. A principal diferença entre ações e serviços é que, ao chamar um serviço, o robô precisa esperar até que o serviço tenha terminado antes de fazer algo mais. Por outro lado, ao chamar uma ação, o seu robô ainda pode continuar fazendo outra coisa enquanto executa a ação.

Há outras diferenças, como uma ação permitindo que você forneça feedback enquanto a ação está sendo realizada.

##  Launch files
Um programa ROS é executado usando alguns arquivos especiais chamados launch files. A estrutura do comando roslaunch é a seguinte: `roslaunch <package_name> <launch_file>`.
* package_name: Especifica o nome do pacote ROS contendo o launch files.
* launch_file:  E o nome do launch file em si.(que está armazenado dentro do pacote).

### Pacote
O ROS utiliza pacotes para organizar seus programas. Você pode pensar em um pacote como todos os arquivos que um programa ROS específico contém; 
* todos os seus arquivos cpp,
* arquivos python,
* arquivos de configuração,
* arquivos de compilação,
* arquivos de lançamento
* arquivos de parâmetros.

Todos esses arquivos no pacote são organizados com a seguinte estrutura:
* pasta launch: Contém arquivos de lançamento.
* pasta src: Arquivos de origem (cpp, python).
* CMakeLists.txt: Lista de regras do cmake para compilação.
* package.xml: Informações do pacote e dependências.

Para acessar qualquer pacote do ROS, o ROS oferece um comando chamado **roscd**. Ao digitar: `roscd <package_name>`, ele o levará ao diretório onde o pacote package_name está localizado.

### Como a launch file funciona?
No launch file, há algumas tags extras para definir parâmetros e redirecionamentos. A tag **param** define um parâmetro no Servidor de Parâmetros, de onde os nós obtêm parâmetros. Muitos nós utilizam parâmetros para evitar modificar o código-fonte, a baixo você pode ver como eles são adicionados.

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


