from datetime import datetime

class LogManager:
    Fatal = "Fatal"
    Error = "Error"
    Warn  = "Warn"
    Info  = "Info"
    Debug = "Debug"

    def gera_log(self, mensagem: str, tipo_log: int):
        # Gera o nome do arquivo de log com a data atual
        data_atual = datetime.now()
        log_file_path = '/home/ubuntu/Desktop/logs/info/Log_Info_' + data_atual.strftime("%d_%m_%Y") + '.log'
        error_log_file_path = '/home/ubuntu/Desktop/logs/erro/Log_Erro_' + data_atual.strftime("%d_%m_%Y") + '.log'

        data_log = str(data_atual.strftime("%d_%m_%Y %H:%M:%S"))
        mensagem_log = data_log+";" + tipo_log + ";" + mensagem

        # Abre o arquivo no modo de adição (append)
        if tipo_log == self.Info or tipo_log == self.Debug:
            try:
                with open(log_file_path, 'a') as file:
                    file.write(mensagem_log + "\n")
                    file.close()
            except PermissionError as e:
                print(f"Erro de permissão: {e}")
            except FileNotFoundError as e:
                print(f"Erro de caminho: {e}")
                    

                
        if tipo_log == self.Fatal or tipo_log == self.Error or tipo_log == self.Warn:
            try:
                with open(error_log_file_path, 'a') as file:
                    file.write(mensagem_log + "\n")
                    file.close()
            except PermissionError as e:
                print(f"Erro de permissão: {e}")
            except FileNotFoundError as e:
                print(f"Erro de caminho: {e}")

            

if __name__ == '__main__':
    log = LogManager()
    log.gera_log("Iniciando giro do Robô por 5 segundos.",log.Info)