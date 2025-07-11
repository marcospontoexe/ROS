#!/usr/bin/env python3

import smbus
from PIL import Image, ImageDraw, ImageFont

class SSD1306:
    # Constantes de comando do SSD1306
    SET_CONTRAST = 0x81
    DISPLAY_ALL_ON_RESUME = 0xA4
    DISPLAY_ALL_ON = 0xA5
    NORMAL_DISPLAY = 0xA6
    INVERT_DISPLAY = 0xA7
    DISPLAY_OFF = 0xAE
    DISPLAY_ON = 0xAF

    def __init__(self):
        """
        Inicializa a instância da classe para controlar o display OLED.
        """
        try:
            # Inicializa o barramento I2C
            self.bus = smbus.SMBus(0)
            self.address = 0x3C
            self.width = 128
            self.height = 64
            # Cria um buffer de memória para armazenar o conteúdo do display
            self.buffer = [0x00] * (self.width * self.height // 8)
            self.init_display()  # Inicializa o display
        except Exception as e:
            print(f"Erro na inicialização do display: {e}")
            # self.clear()

    def write_command(self, command):
        """
        Envia um comando para o display via I2C.
        
        :param command: Código do comando a ser enviado.
        """
        try:
            self.bus.write_byte_data(self.address, 0x00, command)
        except Exception as e:
            print(f"Erro ao enviar comando: {e}")
            # self.clear()

    def write_data(self, data):
        """
        Envia dados ao display em blocos de até 32 bytes.
        
        :param data: Dados a serem escritos no display.
        """
        try:
            for i in range(0, len(data), 32):
                self.bus.write_i2c_block_data(self.address, 0x40, data[i:i+32])
        except Exception as e:
            print(f"Erro ao enviar dados: {e}")
            # self.clear()

    def init_display(self):
        """
        Inicializa o display com os comandos necessários.
        Define o modo de endereçamento, contraste, multiplexação e outras configurações padrão.
        """
        try:
            # Inicialização básica do display
            self.write_command(self.DISPLAY_OFF)  # DISPLAY_OFF
            self.write_command(0xA8)  # Set Multiplex Ratio
            self.write_command(0x3F)  # Multiplex for 128x64
            self.write_command(0xD3)  # Display Offset
            self.write_command(0x00)  # Offset value
            self.write_command(0x40)  # Set Display Start Line
            self.write_command(0xA1)  # Set Segment Re-map
            self.write_command(0xC8)  # COM Output Scan Direction
            self.write_command(0xDA)  # COM Pins Hardware Configuration
            self.write_command(0x12)  # Sequential COM configuration
            self.write_command(self.SET_CONTRAST)  # Set Contrast Control
            self.write_command(0xCF)  # Contrast value
            self.write_command(self.DISPLAY_ALL_ON_RESUME)  # Entire Display ON (RAM content)
            self.write_command(self.NORMAL_DISPLAY)  # Normal Display
            self.write_command(0xD5)  # Set Oscillator Frequency
            self.write_command(0x80)  # Frequency value
            self.write_command(0x8D)  # Enable charge pump regulator
            self.write_command(0x14)  # Charge Pump enable
            self.write_command(self.DISPLAY_ON)  # DISPLAY_ON
        except Exception as e:
            print(f"Erro na inicialização dos comandos do display: {e}")
            # self.clear()

    def clear(self):
        """
        Limpa o buffer do display, preenchendo-o com zeros.
        """
        try:
            self.buffer = [0x00] * (self.width * self.height // 8)
            self.show()
        except Exception as e:
            print(f"Erro ao limpar o display: {e}")

    def show(self):
        """
        Atualiza o conteúdo do display com os dados do buffer.
        Envia os dados página por página (cada página tem 8 linhas de altura).
        """
        try:
            for i in range(0, self.height // 8):
                self.write_command(0xB0 + i)  # Define o endereço da página inicial
                self.write_command(0x00)     # Endereço de coluna baixa
                self.write_command(0x10)     # Endereço de coluna alta
                self.write_data(self.buffer[i * self.width:(i + 1) * self.width])
        except Exception as e:
            print(f"Erro ao atualizar o display: {e}")
            # self.clear()

    def display_text(self, title, content):
        """
        Exibe texto no display com centralização horizontal e vertical.
        
        :param title: Título a ser exibido na parte superior.
        :param content: Conteúdo a ser exibido abaixo do título, centralizado.
        """
        try:
            # Cria uma nova imagem em modo monocromático
            image = Image.new("1", (self.width, self.height))
            draw = ImageDraw.Draw(image)

            # Carrega fontes
            font_large = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 17)
            font_number = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 50)

            # Desenha o título
            title_width, title_height = draw.textsize(title, font=font_large)
            x_title = (self.width - title_width) // 2
            draw.text((x_title, 0), title, font=font_large, fill=255)

            # Desenha o conteúdo centralizado
            content_width, content_height = draw.textsize(content, font=font_number)
            x_content = (self.width - content_width) // 2  # Centraliza horizontalmente
            y_content = (self.height - title_height - content_height) // 2 + title_height  # Centraliza verticalmente
            draw.text((x_content, y_content), content, font=font_number, fill=255)

            # Atualiza o buffer com os pixels da imagem
            pixels = image.load()
            for y in range(self.height):
                for x in range(self.width):
                    if pixels[x, y] > 0:
                        self.buffer[x + (y // 8) * self.width] |= (1 << (y % 8))
                    else:
                        self.buffer[x + (y // 8) * self.width] &= ~(1 << (y % 8))

            self.show()  # Mostra o conteúdo no display
        except Exception as e:
            print(f"Erro ao exibir texto no display: {e}")
            self.clear()

if __name__ == "__main__":
    try:
        # Cria uma instância do display OLED
        OLED = SSD1306()

        # Limpa o display
        OLED.clear()

        # OLED.test_display()

        # Exibe texto
        OLED.display_text("DESTINO:", "1")
        print("Comando enviado para o display.")
    except Exception as e:
        print(f"Erro durante o teste do display: {e}")


