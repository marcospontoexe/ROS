#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
import os
import json
import tf

class MakeWaypointPaths:
    '''Esta classe captura pontos, criando um arquivo json que contem os pontos para criar um rota ponto a ponto
    '''
    def __init__(self):
        self.caminho_rota = "/home/ubuntu/catkin_ws/src/nav_hub/scripts/config/ponto-ponto-demo/"  
 
        self.option = ''
        self.flagOption = False
        self.PontosRotas = []

        # Subscritor para o tópico /joy
        rospy.Subscriber('/waypoint', String, self.waypoint_callback, queue_size=1)
 
    def waypoint_callback(self, msg):
        self.option = msg.data
        self.flagOption = True
            
    def contar_arquivos_json(self, diretorio):
        '''Conta quantos arquivos terminados em ".json" contem no diretório "diretorio"
        '''
        try:
            arquivos = os.listdir(diretorio)
            json_files = [arquivo for arquivo in arquivos if arquivo.endswith(".json")]
            return len(json_files)
        except FileNotFoundError:
            print(f"O diretório {diretorio} não foi encontrado.")
            return 0
        except PermissionError:
            print(f"Sem permissão para acessar {diretorio}.")
            return 0 

    def criarJson(self, nomeRota, pontos):
        """
        Cria um arquivo .json no diretório "self.caminho_rota"

        Parâmetros:
            nomeRota (str): nome do arquivo a ser criado (sem ou com extensão .json)
            pontos (list): lista de listas/tuplas, onde cada elemento é [x, y, w, z]

        Retorna:
            bool: True se criar o arquivo com sucesso, False caso contrário.
        """

        # print(f"nomeRota: {nomeRota}")

        # Se o nome não terminar com ".json", adiciona
        if not nomeRota.lower().endswith(".json"):
            nome_arquivo = nomeRota + ".json"
        else:
            nome_arquivo = nomeRota

        # print(f"nome do arquivo: {nome_arquivo}")

        # Caminho completo do arquivo
        caminho_completo = os.path.join(self.caminho_rota, nome_arquivo)

        # Dados fixos de covariância para a chave "initial"
        covariancia_fixa = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942
        ]

        try:
            # Garante que o diretório existe; se não existir, cria as pastas necessárias
            os.makedirs(self.caminho_rota, exist_ok=True)

            # Montar estrutura do JSON
            dados_json = {}

            # Montar a chave "initial" a partir do ponto de índice 0
            x0, y0, w0, z0 = pontos[0]
            # print(f"x0: {x0}")
            # print(f"y0: {y0}")
            # print(f"w0: {w0}")
            # print(f"z0: {z0}")

            dados_json["initial"] = {
                "sequence": 1,
                "covariance": covariancia_fixa,
                "w": w0,
                "y": y0,
                "x": x0,
                "z": z0
            }

            # Montar a chave "step0" a partir do mesmo ponto inicial (índice 0)
            dados_json["step0"] = {
                "sequence": 2,
                "w": w0,
                "y": y0,
                "x": x0,
                "z": z0
            }

            # Caso haja pontos intermediários (entre o índice 1 e o penúltimo),
            # monta as chaves "step1", "step2", etc. Até o índice N-2
            # O valor de "sequence" vai aumentando sequencialmente
            # Já reservamos sequence=1 para “initial” e sequence=2 para “step0”
            seq_atual = 3  # próxima sequência: 3
            # percorrer índices de 1 até len(pontos)-2 (inclusive) para criar step1, step2, ...
            for idx in range(1, len(pontos) - 1):
                xi, yi, wi, zi = pontos[idx]
                # print(f"idx: {idx}")
                # print(f"xi: {xi}")
                # print(f"yi: {yi}")
                # print(f"wi: {wi}")
                # print(f"zi: {zi}")
                chave_step = f"step{idx}"
                dados_json[chave_step] = {
                    "sequence": seq_atual,
                    "w": wi,
                    "y": yi,
                    "x": xi,
                    "z": zi
                }
                seq_atual += 1

            # Montar a chave "ending" com base no último ponto (índice len(pontos)-1)
            x_last, y_last, w_last, z_last = pontos[-1]
            dados_json["ending"] = {
                "sequence": len(pontos) + 1,
                "w": w_last,
                "y": y_last,
                "x": x_last,
                "z": z_last
            }

            # Gravar em disco com identação de 4 espaços para facilitar leitura
            with open(caminho_completo, "w") as f:
                json.dump(dados_json, f, indent=4)
                f.close()

            return True

        except Exception as e:
            # Caso haja qualquer erro (permissão, diretório inexistente, etc.), retorna False
            # Opcionalmente, pode-se fazer um log: print(f"Erro ao criar JSON: {e}")
            return False
            

    def apagar_jsons(self):
        """
        Acessa o diretório "self.caminho_rota" e apaga todos os arquivos com extensão ".json".

        Retorna:
            True  - se todos os arquivos JSON forem excluídos (ou se não houver .json)
            False - se ocorrer qualquer erro (diretório inacessível, permissão negada, etc.)
        """
        
        try:
            # Verifica se o diretório existe antes de tentar listar
            if not os.path.isdir(self.caminho_rota):
                # Retorna False caso o diretório não exista ou não seja uma pasta
                return False

            # Itera sobre todas as entradas (arquivos e pastas) no diretório
            for nome_arquivo in os.listdir(self.caminho_rota):
                # Verifica se o nome termina com ".json"
                if nome_arquivo.lower().endswith(".json"):
                    # Monta o caminho completo para o arquivo
                    caminho_completo = os.path.join(self.caminho_rota, nome_arquivo)
                    # Remove o arquivo JSON encontrado
                    os.remove(caminho_completo)

            return True

        except Exception:
            # Qualquer erro durante listagem ou remoção resulta em False
            return False

        

    def run(self):
        if self.flagOption:
            if self.option == "capturar":        # captura um ponto
                robot_pose = rospy.wait_for_message('/keyence_pose', PoseWithCovarianceStamped)
                robot_x = robot_pose.pose.pose.position.x
                robot_y = robot_pose.pose.pose.position.y
                robot_w = robot_pose.pose.pose.orientation.w
                robot_z = robot_pose.pose.pose.orientation.z

                self.PontosRotas.append((robot_x, robot_y, robot_w, robot_z))
                print(f"Pontos capturados: {robot_x}, {robot_y}, {robot_w}, {robot_z}")
            elif self.option == "gravar":          # cria uma rota ponto-ponto com os pontos capturados
                if len(self.PontosRotas) > 1:       # craia um arquivo json apenas se existir dois ou mais pontos capturados
                    nomeNovoJson = str(self.contar_arquivos_json(self.caminho_rota) + 1)      # retorna o total de arquivos .json existentes + 1
                    resultCriar = self.criarJson(nomeNovoJson, self.PontosRotas)             # cria um arquivo com o nome do parametros "nomeNovoJson", com o conteudo de "self.PontosRotas"
                    self.PontosRotas = []
                    if resultCriar:
                        print(f"Rota {nomeNovoJson} criada")
                    else:
                        print("Falha ao rota")
            elif self.option == "apagar":             # apaga todos arquivos json dentro do diretório "caminho_rota"
                resultApagar = self.apagar_jsons()
                self.PontosRotas = []
                if resultApagar:
                    print("Rotas apagadas")
                else:
                    print("Falha ao apagar rotas")
            else:
                print(f"Comando incorreto: {self.option}!")
                print(f"Os comando válidos são os seguintes: \n capturar \n gravar \n apagar")
                
            self.flagOption = False
                

if __name__ == '__main__':
    # Inicializa o nó ROS
    rospy.init_node('makeWaypointPaths_node')
    rate = rospy.Rate(1)
    make = MakeWaypointPaths()
    while not rospy.is_shutdown():
        make.run()
        rate.sleep()
    # nav_stack_object.cor_pub.publish("Apagar") #apaga a fita led
