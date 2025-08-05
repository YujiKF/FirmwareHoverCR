// ====================================================================
//    ESP32 Hoverboard PID Controller - Dual Control (Serial + Bluetooth)
// ====================================================================
// Este código permite controlar a velocidade e sintonizar os ganhos do PID
// tanto pelo Monitor Serial do PC como por um terminal Bluetooth (telemóvel).
//
// Comandos (enviar com newline):
//   - S<numero> -> Define a velocidade (ex: S300)
//   - P<numero> -> Define o ganho Kp (ex: P25.5)
//   - I<numero> -> Define o ganho Ki (ex: I12.0)
//   - D<numero> -> Define o ganho Kd (ex: D0.8)
//   - Podem ser enviados múltiplos comandos separados por ';'.
//     Exemplo: S400;P22.0;I11.5;
// ====================================================================

#define _DEBUG      // Ativa a saída de depuração para o Bluetooth
#define ESP32
#define LED_BUILTIN 2

// --- INCLUDES ---
#include "BluetoothSerial.h"
#include "util.h"
#include "hoverserial.h"

// --- CONFIGURAÇÕES DE COMUNICAÇÃO ---
#define HOVER_BAUDRATE 19200
#define BAUDRATE_MONITOR 115200
#define SEND_INTERVAL_MS 50 // Envia um comando para o hoverboard a cada 50ms

const int HOVER_RX_PIN = 16;
const int HOVER_TX_PIN = 17;

// --- OBJETOS DE COMUNICAÇÃO ---
BluetoothSerial SerialBT;                    // Bluetooth para o telemóvel
HardwareSerial SerialHover(1);               // Serial de hardware para o Hoverboard (UART1)
SerialHover2Server oHoverFeedback;           // Estrutura para guardar os dados do hoverboard

// --- VARIÁVEIS DE CONTROLO GLOBAIS ---
volatile int iSpeed = 0;
volatile float Kp_s = 20.0f; 
volatile float Ki_s = 10.0f;
volatile float Kd_s = 0.5f;

unsigned long lastSendTime = 0;
unsigned long lastLogTime = 0;

// ====================================================================
//                  FUNÇÃO PARA PROCESSAR COMANDOS
// ====================================================================
// Esta função processa uma string de comando (vinda do PC ou do telemóvel)
void parseAndSetValues(String input) {
  input.trim();
  if (input.length() == 0) return;

  #ifdef _DEBUG
    Serial.print("Comando recebido: '"); Serial.print(input); Serial.println("'");
  #endif

  int lastPos = 0;
  while (lastPos < input.length()) {
    int sepIndex = input.indexOf(';', lastPos);
    if (sepIndex == -1) sepIndex = input.length();
    String cmd = input.substring(lastPos, sepIndex);
    cmd.trim();
    
    // Converte o comando para maiúsculas para ser insensível a maiúsculas/minúsculas
    cmd.toUpperCase();

    if (cmd.startsWith("S")) {
      iSpeed = cmd.substring(1).toInt();
      String response = "OK: Velocidade definida para " + String(iSpeed);
      SerialBT.println(response);
      #ifdef _DEBUG
        Serial.println(response);
      #endif
    } else if (cmd.startsWith("P")) {
      Kp_s = cmd.substring(1).toFloat();
      String response = "OK: Kp definido para " + String(Kp_s, 4);
      SerialBT.println(response);
      #ifdef _DEBUG
        Serial.println(response);
      #endif
    } else if (cmd.startsWith("I")) {
      Ki_s = cmd.substring(1).toFloat();
      String response = "OK: Ki definido para " + String(Ki_s, 4);
      SerialBT.println(response);
      #ifdef _DEBUG
        Serial.println(response);
      #endif
    } else if (cmd.startsWith("D")) {
      Kd_s = cmd.substring(1).toFloat();
      String response = "OK: Kd definido para " + String(Kd_s, 4);
      SerialBT.println(response);
      #ifdef _DEBUG
        Serial.println(response);
      #endif
    }
    lastPos = sepIndex + 1;
  }
}

// ====================================================================
//                          SETUP
// ====================================================================
void setup() {
  #ifdef _DEBUG
    Serial.begin(BAUDRATE_MONITOR);
    Serial.println("ESP32 Hoverboard - Controle Bluetooth");
    Serial.println("==============================================");
  #endif

  // Inicia o Bluetooth para o telemóvel
  SerialBT.begin("Hoverboard_ESP32");
  #ifdef _DEBUG
    Serial.println("Bluetooth iniciado. Conecte-se a 'Hoverboard_ESP32'");
    Serial.println("Envie comandos via Serial ou Bluetooth (ex: S300; P10.5;)");
  #endif

  // Inicia a porta serial de hardware para o Hoverboard
  HoverSetupEsp32(SerialHover, HOVER_BAUDRATE, HOVER_RX_PIN, HOVER_TX_PIN);
  
  pinMode(LED_BUILTIN, OUTPUT);
}


// ====================================================================
//                            LOOP
// ====================================================================
void loop() {
  unsigned long iNow = millis();
  digitalWrite(LED_BUILTIN, (iNow % 1000) < 100); // Pisca rápido para mostrar que está funcionando

  // 1. Verifica se chegaram comandos do PC (Monitor Serial)
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    parseAndSetValues(input);
  }

  // 2. Verifica se chegaram comandos do Telemóvel (Bluetooth)
  if (SerialBT.available() > 0) {
    String input = SerialBT.readStringUntil('\n');
    parseAndSetValues(input);
  }

  // 3. Verifica se chegaram dados de telemetria do hoverboard
  if (Receive(SerialHover, oHoverFeedback)) {
    // Para não inundar, envia telemetria apenas a cada 100ms
    if (iNow - lastLogTime > 100) {
      String feedbackString = "Tensão:" + String(oHoverFeedback.iVolt / 100.0, 2) + 
                              " | Corrente:" + String(oHoverFeedback.iAmpL / 100.0, 2) + 
                              " | Setpoint:" + String(iSpeed * 0.02676f, 2) + " km/h"
                              " | Velocidade:" + String(oHoverFeedback.iSpeedL / 100.0, 2) + " km/h";
      
      Serial.println(feedbackString);
      #ifdef _DEBUG
        Serial.println(feedbackString);
        SerialBT.println(feedbackString);
      #endif
      lastLogTime = iNow;
    }
  }

  // 4. Envia o comando atual para o hoverboard no intervalo de tempo certo
  if (iNow - lastSendTime > SEND_INTERVAL_MS) {
    HoverSend(SerialHover, 0, iSpeed, 32, 32, Kp_s, Ki_s, Kd_s);
    lastSendTime = iNow;
  }
}