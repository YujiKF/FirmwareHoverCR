template <typename O,typename I> void HoverSetupEsp32(O& oSerial, I iBaud, I gpio_RX, I gpio_TX)
{
  // Starts the serial connection using the baud, protocol, GPIO RX, GPIO TX.
  // These are the GPIO numbers; not necessarily the pin number printed on the PCB.
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

#define START_FRAME         0xABCD       // [-] Start frme definition for reliable serial communication

typedef struct __attribute__((packed, aligned(1))) {
    uint16_t cStart = START_FRAME;    //  = '/';
    int16_t iSpeedL;   // 100* km/h
    int16_t iSpeedR;   // 100* km/h
    uint16_t iVolt;    // 100* V
    int16_t iAmpL;   // 100* A
    int16_t iAmpR;   // 100* A
    int32_t iOdomL;    // hall steps
    int32_t iOdomR;    // hall steps
    uint16_t checksum;
} SerialHover2Server;

//typedef struct{   // new version
//   uint16_t cStart = START_FRAME;   // new version
typedef struct __attribute__((packed, aligned(1))) {  // old version
    uint8_t cStart = '/';                              // old version
    int16_t  iSpeed = 0;
    int16_t  iSteer = 0;
    uint8_t  wStateMaster = 0;   // 1=ledGreen, 2=ledOrange, 4=ledRed, 8=ledUp, 16=ledDown   , 32=Battery3Led, 64=Disable, 128=ShutOff
    uint8_t  wStateSlave = 0;   // 1=ledGreen, 2=ledOrange, 4=ledRed, 8=ledUp, 16=ledDown   , 32=Battery3Led, 64=Disable, 128=ShutOff
    float Kp = 0;
    float Ki = 0;
    float Kd = 0;
    uint16_t checksum;
} SerialServer2Hover;

template <typename O,typename I> void HoverSend(O& oSerial, I iSteer, I iSpeed,uint8_t  wStateMaster=32, uint8_t  wStateSlave=0, float Kp = 0, float Ki = 0, float Kd = 0)
{
  //DEBUGT("iSteer",iSteer);DEBUGN("iSpeed",iSpeed);
  SerialServer2Hover oData;
  oData.iSpeed    = (int16_t)iSpeed;
  oData.iSteer    = (int16_t)iSteer;
  oData.wStateMaster  = wStateMaster;
  oData.wStateSlave   = wStateSlave;
  oData.Kp = Kp;
  oData.Ki = Ki;
  oData.Kd = Kd;
  oData.checksum = CalcCRC((uint8_t*)&oData, sizeof(SerialServer2Hover)-2); // first bytes except crc
  oSerial.write((uint8_t*) &oData, sizeof(SerialServer2Hover)); 
  //DebugOut((uint8_t*) &oData, sizeof(oData)); 
}

void HoverLog(SerialHover2Server& oData)
{
  DEBUGT("iOdomL",oData.iOdomL);
  DEBUGT("\tiOdomR",oData.iOdomR);
  DEBUGT("\tiSpeedL",(float)oData.iSpeedL/100.0);
  DEBUGT(" iSpeedR",(float)oData.iSpeedR/100.0);
  DEBUGT("\tiAmpL",(float)oData.iAmpL/100.0);
  DEBUGT(" iAmpR",(float)oData.iAmpR/100.0);
  DEBUGN("\tiVolt",(float)oData.iVolt/100.0);
}

void HoverDebug(SerialHover2Server& oData)
{
  DEBUGTX("0",oData.iVolt);
  DEBUGTB("1",oData.iAmpL);
  DEBUGTB("2",oData.iAmpR);
  DEBUGTB("3",oData.iSpeedL);
  DEBUGN("4",oData.iSpeedR);
}

void HoverDebug2(SerialHover2Server& oData)
{
  DEBUGT("0",oData.iVolt);
  DEBUGT("1",oData.iAmpL);
  DEBUGT("2",oData.iAmpR);
  DEBUGT("3",oData.iSpeedL);
  DEBUGT("4",oData.iSpeedR);
  DEBUGT("5",oData.iOdomL);
  DEBUGN("6",oData.iOdomR);
}


void DebugOut(uint8_t aBuffer[], uint8_t iSize)
{
  for (int i=0; i<iSize; i++)
  {
    uint8_t c = aBuffer[i];
    Serial.print((c < 16) ? " 0" : " ");Serial.print(c,HEX); 
  }
  Serial.println();
}

#ifdef DEBUG_RX
  unsigned long iLastRx = 0;
#endif

//boolean Receive(Serial& oSerial, SerialFeedback& Feedback)
template <typename O,typename OF> boolean Receive(O& oSerial, OF& Feedback)
{
  int iTooMuch = oSerial.available() - sizeof(SerialHover2Server) + 1;
  int8_t bFirst = 1;
  while (iTooMuch >= bFirst )
  {
    byte c = oSerial.read();  // Read the incoming byte
    iTooMuch--;
    
    if (bFirst) // test first START byte
    {
      if (c == (byte)START_FRAME) //if (c == 0xCD)
      {
        bFirst = 0;
      }
    }
    else  // test second START byte
    {
      if (c == START_FRAME >>8 ) //if (c == 0xAB)
      {
        //DEBUGT(" avail",oSerial.available())
        SerialHover2Server tmpFeedback;
        byte* p = (byte *)&tmpFeedback+2; // start word already read
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
      if (c != (byte)START_FRAME) //if (c != 0xCD)
        bFirst = 1;
    }
  }
  return false;
}
