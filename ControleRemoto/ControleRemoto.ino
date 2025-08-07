/*
 * ====================================================================
 * Firmware Final para Controle Remoto
 * ====================================================================
*/

// --- INCLUDES ---
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

// --- CONFIGURAÇÕES DE HARDWARE ---
#define I2C_ADDRESS 0x3C
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_SDA_PIN 21
#define OLED_SCL_PIN 22
#define ENCODER_CLK_PIN 25
#define ENCODER_DT_PIN  26
#define ENCODER_SW_PIN  27
#define MODE_SWITCH_ECO_PIN   13
#define MODE_SWITCH_SPORT_PIN 18

// Pino para leitura da bateria do controle 
#define BAT_FEEDBACK_PIN 33 // Conforme o esquemático (BAT_FEEDBACK -> IO33)

// --- CONFIGURAÇÕES DE COMUNICAÇÃO ESP-NOW ---
#define WIFI_CHANNEL 1
// Endereço MAC da ESP32 do Hoverboard
uint8_t hoverboardMacAddress[] = {0x80, 0x7D, 0x3A, 0xF9, 0x70, 0xE4};
const float SPEED_TO_KMH_FACTOR = 0.02676f;

// Estruturas de Dados
typedef struct struct_command { int speed; float kp; float ki; float kd; } struct_command;
struct_command commandData;
typedef struct struct_telemetry { float real_speed_kmh; float battery_volt; } struct_telemetry;
struct_telemetry incomingTelemetry;

esp_now_peer_info_t peerInfo;
volatile bool packetSentSuccess = true;
volatile bool packetReceived = false;

// --- OBJETOS GLOBAIS E VARIÁVEIS ---
Adafruit_SH1106G display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
enum EditMode { MODE_SPEED, MODE_KP, MODE_KI, MODE_KD };
volatile EditMode currentEditMode = MODE_SPEED;
enum DriveMode { ECO, SPORT };
DriveMode currentDriveMode = ECO;
float Kp_eco = 30.0f, Ki_eco = 80.0f, Kd_eco = 5.0f;
float Kp_sport = 40.0f, Ki_sport = 100.0f, Kd_sport = 5.5f;
volatile long encoderPos = 0, lastEncoderPos = 0;
volatile bool encoderButtonPressed = false;
float   current_kp = Kp_eco, current_ki = Ki_eco, current_kd = Kd_eco;
unsigned long lastSendTime = 0, lastDisplayTime = 0, lastBatReadTime = 0;

// Variável para a bateria do controle remoto
float remote_battery_volt = 0.0;

// --- FUNÇÕES DE CALLBACK (ESP-NOW) ---
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  packetSentSuccess = (status == ESP_NOW_SEND_SUCCESS);
}
void OnDataRecv(const esp_now_recv_info_t * mac_info, const uint8_t *incomingData, int len) {
  memcpy(&incomingTelemetry, incomingData, sizeof(incomingTelemetry));
  packetReceived = true;
}

// --- FUNÇÕES DE INTERRUPÇÃO (ENCODER) ---
void IRAM_ATTR readEncoder() {
  if (digitalRead(ENCODER_DT_PIN) != digitalRead(ENCODER_CLK_PIN)) { encoderPos++; } else { encoderPos--; }
}
void IRAM_ATTR readEncoderButton() {
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
  if (interrupt_time - last_interrupt_time > 250) { encoderButtonPressed = true; }
  last_interrupt_time = interrupt_time;
}

// ====================================================================
//                    FUNÇÃO DO DISPLAY (MODIFICADA)
// ====================================================================
void updateDisplay() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  
  // --- Linha Superior: Modo e Baterias ---
  display.setCursor(0, 0);
  display.print("Modo: ");
  display.print(currentDriveMode == SPORT ? "ESPOR" : "ECO");

  // >>>>> NOVIDADE AQUI: Exibe as duas baterias <<<<<
  display.setCursor(68, 0); 
  display.print("HB:");
  display.print(incomingTelemetry.battery_volt, 1);
  
  display.setCursor(68, 10);
  display.print("RC:");
  display.print(remote_battery_volt, 1);
  // ----------------------------------------------
  
  display.setCursor(0, 10);
  display.println("---------");

  // --- Velocidades ---
  display.setCursor(0, 20);
  display.print(currentEditMode == MODE_SPEED ? ">" : " ");
  display.print(" Set: ");
  display.print(commandData.speed * SPEED_TO_KMH_FACTOR, 1);
  display.println(" km/h");

  display.print("  Real: ");
  display.print(incomingTelemetry.real_speed_kmh, 1);
  display.println(" km/h");

  // --- Ganhos PID ---
  display.setCursor(0, 48);
  display.print(currentEditMode == MODE_KP ? ">" : " ");
  display.print("P:"); display.print(current_kp, 2);
  
  display.setCursor(64, 48);
  display.print(currentEditMode == MODE_KI ? ">" : " ");
  display.print("I:"); display.print(current_ki, 2);
  
  display.setCursor(0, 56);
  display.print(currentEditMode == MODE_KD ? ">" : " ");
  display.print("D:"); display.print(current_kd, 2);

  // --- Indicador de Falha de Transmissão ---
  if(!packetSentSuccess) {
    display.setCursor(90, 56);
    display.print("[TX!]");
  }

  display.display();
}

// ====================================================================
//                               SETUP
// ====================================================================
void setup() {
  Serial.begin(115200);
  Wire.begin(OLED_SDA_PIN, OLED_SCL_PIN);
  pinMode(BAT_FEEDBACK_PIN, INPUT); // Configura o pino da bateria como entrada

  if(!display.begin(I2C_ADDRESS, true)) { for(;;); }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0,0);
  display.println("Controle Hoverboard");
  display.println("Iniciando...");
  display.display();

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  if (esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE) != ESP_OK) { return; }
  if (esp_now_init() != ESP_OK) { return; }
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  memcpy(peerInfo.peer_addr, hoverboardMacAddress, 6);
  peerInfo.channel = WIFI_CHANNEL;  
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK){ return; }
  
  display.println("Pronto!");
  display.display();
  delay(1000);

  pinMode(MODE_SWITCH_ECO_PIN, INPUT_PULLDOWN);
  pinMode(MODE_SWITCH_SPORT_PIN, INPUT_PULLDOWN);
  pinMode(ENCODER_SW_PIN, INPUT_PULLUP);
  pinMode(ENCODER_CLK_PIN, INPUT_PULLUP);
  pinMode(ENCODER_DT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_CLK_PIN), readEncoder, FALLING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_SW_PIN), readEncoderButton, FALLING);
}

// ====================================================================
//                            LOOP PRINCIPAL
// ====================================================================
void loop() {
  unsigned long iNow = millis();
  if (packetReceived) { packetReceived = false; }
  
  // --- NOVIDADE AQUI: Leitura da bateria do controle a cada 1 segundo ---
  if (iNow - lastBatReadTime > 1000) {
    // Lê o valor analógico (0-4095)
    int adc_raw = analogRead(BAT_FEEDBACK_PIN);
    
    // Converte o valor ADC para a voltagem no pino (considerando 3.3V como referência)
    float adc_volt = (adc_raw / 4095.0) * 3.3;

    // Converte a voltagem do pino para a voltagem real da bateria,
    // compensando o divisor de tensão (R1=2.2k, R2=5.6k)
    // V_bat = V_adc * (R1+R2)/R2 = V_adc * (2200+5600)/5600 = V_adc * 1.3928
    remote_battery_volt = adc_volt * 1.3928;

    lastBatReadTime = iNow;
  }
  // --------------------------------------------------------------------

  if (digitalRead(MODE_SWITCH_SPORT_PIN) == HIGH) {
    if (currentDriveMode != SPORT) { currentDriveMode = SPORT; current_kp = Kp_sport; current_ki = Ki_sport; current_kd = Kd_sport; }
  } else {
    if (currentDriveMode != ECO) { currentDriveMode = ECO; current_kp = Kp_eco; current_ki = Ki_eco; current_kd = Kd_eco; }
  }
  if (encoderButtonPressed) { currentEditMode = (EditMode)(((int)currentEditMode + 1) % 4); encoderButtonPressed = false; }
  if (encoderPos != lastEncoderPos) {
    long delta = encoderPos - lastEncoderPos;
    switch (currentEditMode) {
      case MODE_SPEED: commandData.speed += (delta * 5); commandData.speed = constrain(commandData.speed, 0, 1000); break;
      case MODE_KP: current_kp += (delta * 0.25f); current_kp = constrain(current_kp, 0, 100); break;
      case MODE_KI: current_ki += (delta * 0.25f); current_ki = constrain(current_ki, 0, 100); break;
      case MODE_KD: current_kd += (delta * 0.05f); current_kd = constrain(current_kd, 0, 10); break;
    }
    lastEncoderPos = encoderPos;
  }
  if (iNow - lastSendTime > 100) {
    commandData.kp = current_kp; commandData.ki = current_ki; commandData.kd = current_kd;
    esp_now_send(hoverboardMacAddress, (uint8_t *) &commandData, sizeof(commandData));
    lastSendTime = iNow;
  }
  if (iNow - lastDisplayTime > 100) { updateDisplay(); lastDisplayTime = iNow; }
}