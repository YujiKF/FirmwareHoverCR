# Objetivo

Esse repositório reúne os diversos firmwares desenvolvidos na Iniciação Científica do Propulsor elétrico auxiliar para cadeira de rodas no Laboratório de Sistemas Embarcados do Departamento de Engenharia Mecatrônica e Sistemas Mecânicos da Escola Politécnica da USP.

# Divisão de placas
O sistema foi divido em três módulos:

- Potência: A placa de segunda geração do hoverboard, que contém
um microcontrolador GD32, drivers de motor, transistores e sensores
do tipo Hall para odometria. Ela é responsável por controlar o motor
utilizado no projeto, assim como enviar dados de telemetria para o
microcontrolador ESP32 conectado a ela via interface serial (UART).
É possível encontrar o firmware desenvolvido [aqui.](https://github.com/YujiKF/FirmwareHoverCR/tree/main/HoverBoardGigaDevice)
- Comunicação: O microcontrolador ESP32 é responsável por receber
os dados da placa de potência e enviar através de um cabo USB para
um computador, por bluetooth e para uma outra ESP32, utilizada
no controle remoto, através do protoco de comunicação ESP-NOW.
Também é possível encontrar o firmware desenvolvido [aqui.](https://github.com/YujiKF/FirmwareHoverCR/tree/main/HoverboardSerialESP32)
- Interface: É o controle remoto, isto é, a parte em que o usuário tem
acesso, é uma placa ESP32 equipada com um encoder rotativo com
botão, um display OLED, uma bateria interna com monitoração de
tensão e uma chave seletora para alternar entre modo de pilotagem
econômica ou esportiva. No modo esportivo, os ganhos do PID são
acentuados, fazendo com que mais bateria seja gasta ao custo de ser
mais agressivo e rápido, enquanto isso, no modo econômico o sistema
é mais suave, além de poupar a bateria. Seu firmware está [aqui.](https://github.com/YujiKF/FirmwareHoverCR/tree/main/Arduino)

# Tutoriais

É possível encontrar na [Wiki aqui](https://github.com/YujiKF/FirmwareHoverCR/wiki) todos os tutoriais de utilização.


