# PROJETO SEGUIDOR DE PAREDE
Este projeto foi desenvolvido para obter a certificação do curso [**ROS Basics in 5 Days**](https://app.theconstruct.ai/courses/55).

## O projeto
Este projeto é composto por três partes. 

* **SEÇÃO 1: Publisher e subscribers de tópicos**: Esta seção foca em exercícios práticos com tópicos ROS, incluindo publicação e assinatura de mensagens de dados.
* **SEÇÃO 2: Serviços**: Nesta seção, você ganhará experiência trabalhando com serviços ROS, que permitem solicitações e respostas pontuais entre nós.
* **SEÇÃO 3: Ações**: A seção final irá guiá-lo pelo uso de ações ROS, adequadas para tarefas mais complexas que envolvem comunicação contínua e feedback entre nós.

## Tópics
O objetivo é criar um programa ROS que faça o robô seguir a parede. 

### Comportamento do seguidor de paredes
O comportamento de seguir a parede faz com que o robô se mova ao longo da parede do seu lado direito. Isso significa que o robô deve se mover para frente a uma distância de 30 cm da parede, mantendo a parede sempre à sua direita.

### Implementação
Para alcançar esse comportamento no robô, você precisa fazer duas coisas:

1. **Assinar o tópico do laser do robô**: Você precisa se inscrever no tópico do laser e capturar os raios. Selecione o raio à direita (aquele que faz um ângulo de 90º à direita com a frente do robô) e use-o para medir a distância do robô à parede.

**NOTA:** Os tópicos para diferentes simulações provavelmente não terão o mesmo nome. Portanto, certifique-se de usar o nome correto. O tópico do laser nesta simulação é `/scan`.

2. **Publicar no tópico de velocidade do robô**:
    - Se a distância do raio for maior que 0,3 m, você precisa fazer o robô se aproximar um pouco da parede, adicionando alguma velocidade rotacional ao robô.
    - Se a distância do raio for menor que 0,2 m, você precisa afastar o robô da parede, adicionando velocidade rotacional na direção oposta.
    - Se a distância do raio estiver entre 0,2 m e 0,3 m, mantenha o robô se movendo para frente.

**IMPORTANTE:** Quando o robô está se movendo ao longo de uma parede, ele pode encontrar outra parede em seu caminho. Nesse momento, você deve incluir um comportamento que faça a transição progressiva do robô de seguir a parede atual para a próxima.

Para isso, recomendamos usar o raio frontal do laser. Se a distância medida por esse raio for menor que 0,5 m, faça o robô girar rapidamente para a esquerda (continuando em frente ao mesmo tempo).

## Services
Crie um novo nó ROS que contenha um servidor de serviço chamado `find_wall`. O servidor usa uma mensagem `FindWall.srv` que você deve criar:
```

---
bool wallfound
```

Quando o serviço for chamado, o robô deve realizar o seguinte comportamento:
1. Identifique qual raio laser é o mais curto. Suponha que este é o que está apontando para uma parede.
2. Gire o robô até que a frente dele esteja voltada para a parede. Isso pode ser feito publicando uma velocidade angular até que o raio frontal seja o menor.
3. Mova o robô para frente até que o raio frontal seja menor que 30 cm.
4. Agora, gire o robô novamente até que o raio número 270 das medições do laser esteja apontando para a parede.
5. Nesse ponto, considere que o robô está pronto para começar a seguir a parede.
6. Retorne a mensagem de serviço com `True`.

Adicione a chamada do servidor ao programa da Seção 1:
1. Adicione um cliente de serviço ao nó da Seção 1.
2. Chame o serviço antes do laço de controle desse nó, para que o robô se prepare autonomamente antes de começar a seguir a parede.

Crie um novo arquivo de lançamento (launch) chamado main.launch que inicie ambos os nós: primeiro o nó do servidor de serviço e depois o nó de seguir parede. Isso deve iniciar o movimento do robô (primeiro o robô procura a parede e depois começa a segui-la).

## Actions
O objetivo desta seção é criar um servidor de ação que registra a odometria do robô:
1. Crie um servidor de ação que, quando chamado, comece a gravar a odometria.
2. Adicione uma chamada ao servidor de ação a partir do nó de seguir parede.
3. Inclua o lançamento do servidor de ação no arquivo main.launch.

### Crie um servidor de ações
* Crie um novo nó que contenha um servidor de ação chamado **record_odom**. O servidor utiliza uma mensagem **OdomRecord.action** que você também deve criar:
```
OdomRecord.action
---
geometry_msgs/Point32[] list_of_odoms
---
float32 current_total
```

* O servidor deve começar a gravar a odometria (x, y, theta) do robô como um **Point32**, uma medida a cada segundo.
* Como **feedback**, o servidor de ação deve fornecer a quantidade total de metros que o robô percorreu até o momento.
* Quando o robô completar uma volta completa, o servidor de ação deve terminar e retornar a lista das odometrias registradas.

### Adicione uma chama para o servidor de ações
Adicione um cliente de serviço de ação ao nó da Seção 1. Chame o servidor de ação antes do laço de controle desse nó, para que o robô comece a gravar a odometria antes de começar a seguir a parede.

