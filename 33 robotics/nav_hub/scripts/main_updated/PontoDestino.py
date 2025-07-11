#!/usr/bin/env python3
import os

class PontoDestino:        
    def __init__(self):
        self.nome_deretorio = "/home/ubuntu/catkin_ws/src/nav_hub/scripts/main_updated/ponto_destino"
    def abreArquivo(self, nome_arquivo):
        """
        Abre o arquivo e retorna o valor inteiro contido nele.

        Parâmetros:
        nome_arquivo (str): Nome do arquivo a ser lido.

        Retorna:
        int: Valor contido no arquivo, ou 0 se o arquivo não existir ou houver erro.
        """
        # Caminho do subdiretório e arquivo
        subdiretorio = os.path.join(self.nome_deretorio)  # Usa o separador correto para o SO
        arquivo_txt = os.path.join(subdiretorio, nome_arquivo + '.txt')
        
        try:
            with open(arquivo_txt, "rt") as arquivo:
                valor = int(arquivo.read().strip())
                arquivo.close()
                return valor
        except FileNotFoundError:
            print(f"O arquivo {nome_arquivo} não existe!")
            return 1
        except ValueError:
            print(f"O arquivo {nome_arquivo} não contém um valor inteiro válido!")
            return 1

    def salvaArquivo(self, nome_arquivo, valor):
        """
        Salva o valor inteiro em um arquivo.

        Parâmetros:
        nome_arquivo (str): Nome do arquivo onde o valor será salvo.
        valor (int): Valor inteiro a ser salvo no arquivo.

        Retorna:
        int: 1 se o arquivo foi salvo com sucesso, 0 caso contrário.
        """
        # Caminho do subdiretório e arquivo
        # Caminho do subdiretório e arquivo
        subdiretorio = os.path.join(self.nome_deretorio)  # Usa o separador correto para o SO
        arquivo_txt = os.path.join(subdiretorio, nome_arquivo + '.txt')
        # Verifica se o subdiretório existe, se não, cria-o
        if not os.path.exists(subdiretorio):
            os.makedirs(subdiretorio)
        try:
            with open(arquivo_txt, "wt+") as arquivo:    # Escrita no arquivo (apaga o conteúdo existente), caso não exista cria um arquivo
                try:
                    arquivo.write(str(valor))   #tenta escrever (append) no arquivo txt
                    # print(f"valor salvo: {valor}")
                    arquivo.close()
                    return True
                except Exception as e:
                    print(f"Não foi possível escrever no arquivo {nome_arquivo}: {e}")
                    arquivo.close()
                    return False 
        except Exception as e:
            print(f"Não foi possível salvar o arquivo {nome_arquivo}: {e}")
            return False

# Exemplo de uso da classe
if __name__ == "__main__":
    ponto = PontoDestino()


