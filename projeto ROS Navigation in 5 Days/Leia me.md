# O projeto

Este rosject é composto por quatro seções. Você deve realizar as quatro primeiras seções conforme indicado no
 curso **ROS Navigation in 5 Days**. 

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
