/*
 * ====================================================================
 * Firmware Final e Unificado para Hoverboard
 * ====================================================================
 * ARQUIVO ÚNICO
 *
 * Este código é para a ESP32 que fica conectada ao HOVERBOARD. O pino 16 da ESP32 (RX) deve ser conectado ao pino TX do Hoverboard, 
 assim como o pino 17 da ESP32 (TX) deve ser conectado ao pino RX do Hoverboard, além disso, os GNDs devem ser conectados também.
 * ====================================================================
*/

// --- INCLUDES NECESSÁRIOS PARA O CÓDIGO PRINCIPAL ---
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

template <typename O,typename I> void HoverSetupEsp32(O& oSerial, I iBaud, I gpio_RX, I gpio_TX)
{
  // Inicia a comunicação serial usando baudrate, protocolo, GPIO RX, GPIO TX.
  // Números de GPIO da placa ESP32.
  oSerial.begin(iBaud, SERIAL_8N1, gpio_RX, gpio_TX);
}

uint16_t CalcCRC(uint8_t *ptr, int count)
{
  uint16_t  crc;
  uint8_t i;
  crc = 0;
  while (--count >= 0)
  {
    crc = crc ^ (uint16_t) *ptr++ << 8;
    i = 8;
    do
    {
      if (crc & 0x8000)
      {
        crc = crc << 1 ^ 0x1021;
      }
      else
      {
        crc = crc << 1;
      }
    } while(--i);
  }
  return (crc);
}

#define START_FRAME         0xABCD       // Definição de start frame para comunicação serial

typedef struct __attribute__((packed, aligned(1))) {
    uint16_t cStart = START_FRAME;
    int16_t iSpeedL;
    int16_t iSpeedR;
    uint16_t iVolt;
    int16_t iAmpL;
    int16_t iAmpR;
    int32_t iOdomL;
    int32_t iOdomR;
    uint16_t checksum;
} SerialHover2Server;

typedef struct __attribute__((packed, aligned(1))) {
    uint8_t cStart = '/';
    int16_t  iSpeed = 0;
    int16_t  iSteer = 0;
    uint8_t  wStateMaster = 0;
    uint8_t  wStateSlave = 0;
    float Kp = 0;
    float Ki = 0;
    float Kd = 0;
    uint16_t checksum;
} SerialServer2Hover;

template <typename O,typename I> void HoverSend(O& oSerial, I iSteer, I iSpeed,uint8_t  wStateMaster=32, uint8_t  wStateSlave=0, float Kp = 0, float Ki = 0, float Kd = 0)
{
  SerialServer2Hover oData;
  oData.iSpeed    = (int16_t)iSpeed;
  oData.iSteer    = (int16_t)iSteer;
  oData.wStateMaster  = wStateMaster;
  oData.wStateSlave   = wStateSlave;
  oData.Kp = Kp;
  oData.Ki = Ki;
  oData.Kd = Kd;
  oData.checksum = CalcCRC((uint8_t*)&oData, sizeof(SerialServer2Hover)-2);
  oSerial.write((uint8_t*) &oData, sizeof(SerialServer2Hover)); 
}

template <typename O,typename OF> boolean Receive(O& oSerial, OF& Feedback)
{
  int iTooMuch = oSerial.available() - sizeof(SerialHover2Server) + 1;
  int8_t bFirst = 1;
  while (iTooMuch >= bFirst )
  {
    byte c = oSerial.read();
    iTooMuch--;
    
    if (bFirst) 
    {
      if (c == (byte)START_FRAME)
      {
        bFirst = 0;
      }
    }
    else
    {
      if (c == START_FRAME >>8 )
      {
        SerialHover2Server tmpFeedback;
        byte* p = (byte *)&tmpFeedback+2;
        for (int i = sizeof(SerialHover2Server); i>2; i--)  
          *p++    = oSerial.read();

        uint16_t checksum = CalcCRC((byte *)&tmpFeedback, sizeof(SerialHover2Server)-2);
        if (checksum == tmpFeedback.checksum)
        {
            memcpy(&Feedback, &tmpFeedback, sizeof(SerialHover2Server));
            return true;
        }
        return false;       
      }
      if (c != (byte)START_FRAME)
        bFirst = 1;
    }
  }
  return false;
}

// =================================================================================
// |                INÍCIO DA LÓGICA PRINCIPAL DO HOVERBOARD                       |
// =================================================================================

// --- CONFIGURAÇÕES ---
#define WIFI_CHANNEL 1
#define HOVER_BAUDRATE 19200
#define SEND_INTERVAL_MS 50
const int HOVER_RX_PIN = 16;
const int HOVER_TX_PIN = 17;
#define LED_BUILTIN 2

// --- ESTRUTURAS DE DADOS (ESP-NOW) ---
typedef struct struct_command {int speed; float kp; float ki; float kd; } struct_command;
struct_command incomingCommand;
typedef struct struct_telemetry {float real_speed_kmh; float battery_volt; } struct_telemetry;
struct_telemetry telemetryData;

// --- VARIÁVEIS GLOBAIS ---
HardwareSerial SerialHover(1);
SerialHover2Server oHoverFeedback;
esp_now_peer_info_t peerInfo;
bool isRemotePaired = false;
volatile int iSpeed = 0;
volatile float Kp_s = 20.0f;
volatile float Ki_s = 10.0f;
volatile float Kd_s = 0.5f;
unsigned long lastSendTime = 0, lastLogTime = 0, lastCmdReceivedTime = 0;

// --- FUNÇÃO DE CALLBACK (ESP-NOW) ---
void OnDataRecv(const esp_now_recv_info_t * mac_info, const uint8_t *incomingData, int len) {
  const uint8_t* mac = mac_info->src_addr;
  memcpy(&incomingCommand, incomingData, sizeof(incomingCommand));
  
  // Atualiza as variáveis de controle com os dados recebidos
  iSpeed = incomingCommand.speed;
  Kp_s = incomingCommand.kp;
  Ki_s = incomingCommand.ki;
  Kd_s = incomingCommand.kd;
  lastCmdReceivedTime = millis();

  // Se o controle ainda não foi pareado, faz o pareamento até ter sucesso
  if (!isRemotePaired) {
    memcpy(peerInfo.peer_addr, mac, 6);
    peerInfo.channel = WIFI_CHANNEL;  
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) == ESP_OK){
      isRemotePaired = true;
    }
  }
}

// --- SETUP ---
void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  
  // Configura a comunicação Serial com a placa-mãe do hoverboard
  HoverSetupEsp32(SerialHover, HOVER_BAUDRATE, HOVER_RX_PIN, HOVER_TX_PIN);

  // Configura a comunicação sem fio via ESP-NOW
  WiFi.mode(WIFI_STA);
  delay(500); // Atraso para estabilização do rádio

  if (esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE) != ESP_OK) { return; }
  if (esp_now_init() != ESP_OK) { return; }
  
  esp_now_register_recv_cb(OnDataRecv);
}

// --- LOOP PRINCIPAL---
void loop() {
  unsigned long iNow = millis();
  digitalWrite(LED_BUILTIN, (iNow % 1000) < 50); // Pisca o LED integrado para indicar que está a funcionar

  // Medida de segurança: se não receber comando do controle por mais de 1 segundo, para os motores.
  if (iNow - lastCmdReceivedTime > 1000 && iSpeed != 0) {
    iSpeed = 0;
  }

  // Envia o comando de velocidade/PID para a placa-mãe do hoverboard
  if (iNow - lastSendTime > SEND_INTERVAL_MS) {
    HoverSend(SerialHover, 0, iSpeed, 32, 32, Kp_s, Ki_s, Kd_s);
    lastSendTime = iNow;
  }

  // Recebe dados de telemetria da placa-mãe do hoverboard
  if (Receive(SerialHover, oHoverFeedback)) {
    // Envia a telemetria de volta para o controle remoto
    if (iNow - lastLogTime > 100) {
      telemetryData.real_speed_kmh = oHoverFeedback.iSpeedL / 100.0;
      telemetryData.battery_volt = oHoverFeedback.iVolt / 100.0;
      
      if (isRemotePaired) {
        esp_now_send(peerInfo.peer_addr, (uint8_t *) &telemetryData, sizeof(telemetryData));
      }
      lastLogTime = iNow;
    }
  }
}