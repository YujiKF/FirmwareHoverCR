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

#define START_FRAME         0xABCD       /// Definição de start frame para comunicação serial

typedef struct __attribute__((packed, aligned(1))) {
    uint16_t cStart = START_FRAME;    //  = '/';
    int16_t iSpeedL;   // 100* Velocidade do motor da esquerda em km/h
    int16_t iSpeedR;   // 100* Velocidade do motor da direita em km/h
    uint16_t iVolt;    // 100* Tensão da bateria
    int16_t iAmpL;   // 100* Corrente do motor da esquerda em A
    int16_t iAmpR;   // 100* Corrente do motor da direita em A
    int32_t iOdomL;    // passos do sensor hall do motor da esquerda
    int32_t iOdomR;    // passos do sensor hall do motor da direita
    uint16_t checksum;
} SerialHover2Server;

typedef struct __attribute__((packed, aligned(1))) { 
    uint8_t cStart = '/';                              
    int16_t  iSpeed = 0;         // Velocidade enviada, varia entre -1000 e +1000. Representa a porcentagem de PWM.
    int16_t  iSteer = 0;         // Direção enviada, varia entre -1000 e +1000. Indica quanto o hoverboard "vira", não se aplica ao projeto do propulsor.
    uint8_t  wStateMaster = 0;   // 1=ledVerde, 2=ledLaranja, 4=ledVermelho, 8=ledUp, 16=ledDown, 32=Battery3Led, 64=Disable, 128=ShutOff
    uint8_t  wStateSlave = 0;    // 1=ledVerde, 2=ledLaranja, 4=ledVermelho, 8=ledUp, 16=ledDown, 32=Battery3Led, 64=Disable, 128=ShutOff
    float Kp = 0;                // Ganho proporcional do PID.
    float Ki = 0;                // Ganho integrativo do PID.
    float Kd = 0;                // Ganho derivativo do PID.
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
  oData.checksum = CalcCRC((uint8_t*)&oData, sizeof(SerialServer2Hover)-2); // Primeiros bytes, com exceção do CRC.
  oSerial.write((uint8_t*) &oData, sizeof(SerialServer2Hover)); 
}

template <typename O,typename OF> boolean Receive(O& oSerial, OF& Feedback)
{
  int iTooMuch = oSerial.available() - sizeof(SerialHover2Server) + 1;
  int8_t bFirst = 1;
  while (iTooMuch >= bFirst )
  {
    byte c = oSerial.read();  // Lê o byte recebido
    iTooMuch--;
    
    if (bFirst) // Testa o primeiro START byte
    {
      if (c == (byte)START_FRAME) // Se (c == 0xCD)
      {
        bFirst = 0;
      }
    }
    else  // Testa o segundo START byte
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
      if (c != (byte)START_FRAME) // Se (c != 0xCD)
        bFirst = 1;
    }
  }
  return false;
}
