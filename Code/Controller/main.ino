#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <math.h>
// Audio
#define AUDIO_PIN       3
#define AUDIO_CHANNEL   2        // LEDC channel (0,1 used by display — use 2)
#define AUDIO_FREQ_HZ   200000   // PWM carrier frequency
#define AUDIO_RES_BITS  8        // 8-bit resolution

// Display pins
#define TFT_CS    10
#define TFT_RST    9
#define TFT_DC     8
#define TFT_MOSI  11
#define TFT_CLK   12
#define TFT_MISO  13

// Joystick — Y axis = speed
#define JOY_Y      1
#define JOY_BTN    4

// Buttons — directions
#define BTN_FWD    6
#define BTN_BACK   5
#define BTN_LEFT   7
#define BTN_RIGHT  15

// Buzzer
#define BUZZER    16

// BLE UUIDs
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_MOSI, TFT_CLK, TFT_RST, TFT_MISO);

// Colours
#define BLACK     0x0000
#define WHITE     0xFFFF
#define GREEN     0x07E0
#define RED       0xF800
#define BLUE      0x001F
#define CYAN      0x07FF
#define YELLOW    0xFFE0
#define DARKGREY  0x4208
#define ORANGE    0xFD20

// Screen
#define SCREEN_W  240
#define SCREEN_H  320

// Audio
hw_timer_t *audioTimer  = NULL;
volatile uint32_t audioSampleIndex = 0;
uint8_t sineTable[100];
bool audioPlaying = false;

// BLE
BLECharacteristic *pCharacteristic;
bool deviceConnected    = false;
bool oldDeviceConnected = false;

// Previous values
int prevSpeed = -1;
String prevDirection = "";
bool prevBtns[4] = {false, false, false, false};
String lastCmd = "";

void IRAM_ATTR onAudioTimer() {
  if (audioPlaying) {
    ledcWrite(AUDIO_PIN, sineTable[audioSampleIndex]);
    audioSampleIndex = (audioSampleIndex + 1) % 100;
  } else {
    ledcWrite(AUDIO_PIN, 128);   // DC midpoint — silent
  }
}

void setupAudio() {
  // Build sine table
  for (int i = 0; i < 100; i++) {
    float angle = 2.0f * M_PI * i / 100;
    sineTable[i] = (uint8_t)(128 + 120 * sinf(angle));
  }

  // Setup LEDC
  ledcAttach(AUDIO_PIN, AUDIO_FREQ_HZ, AUDIO_RES_BITS);
  ledcWrite(AUDIO_PIN, 128);

  // Setup timer — 44000 samples/sec = 440Hz tone
  audioTimer = timerBegin(1000000);
  timerAttachInterrupt(audioTimer, &onAudioTimer);
  timerAlarm(audioTimer, 1000000 / 44000, true, 0);

  Serial.println("Audio OK");
}

void beepAudio(int durationMs) {
  audioSampleIndex = 0;
  audioPlaying     = true;
  delay(durationMs);
  audioPlaying     = false;
  ledcWrite(AUDIO_PIN, 128);   // Return to midpoint
}

// ==============================
// BLE Callbacks
// ==============================
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    deviceConnected = true;
    Serial.println("BLE Connected");
  }
  void onDisconnect(BLEServer *pServer) {
    deviceConnected = false;
    Serial.println("BLE Disconnected");
  }
};

void setupBLE() {
  BLEDevice::init("ESP32-Controller");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ  |
    BLECharacteristic::PROPERTY_WRITE |
    BLECharacteristic::PROPERTY_NOTIFY
  );
  pCharacteristic->addDescriptor(new BLE2902());
  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  BLEDevice::startAdvertising();
  Serial.println("BLE advertising started");
}

void sendCommand(String dir, int spd) {
  if (!deviceConnected) return;
  uint32_t duty = map(spd, 0, 100, 0, 1023);
  String cmd = dir + ":" + String(duty);
  pCharacteristic->setValue(cmd.c_str());
  pCharacteristic->notify();
  Serial.printf("BLE TX: %s\n", cmd.c_str());
}

// ==============================
// Buzzer
// ==============================
void startupBeep() {
  for (int i = 0; i < 3; i++) {
    digitalWrite(BUZZER, HIGH);
    delay(100);
    digitalWrite(BUZZER, LOW);
    delay(100);
  }
}

// ==============================
// Display Draw Functions
// ==============================
void drawHeader() {
  tft.fillRect(0, 0, SCREEN_W, 45, BLUE);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.setCursor(10, 8);
  tft.print("ESP32 Controller");
  tft.setTextSize(1);
  tft.setTextColor(CYAN);
  tft.setCursor(10, 30);
  tft.print("DIR: Buttons   SPD: Joystick");
}

void drawDirectionPanel() {
  tft.fillRoundRect(10, 55, SCREEN_W - 20, 85, 8, DARKGREY);
  tft.setTextColor(CYAN);
  tft.setTextSize(1);
  tft.setCursor(20, 63);
  tft.print("DIRECTION");
  tft.setTextSize(4);
  tft.setTextColor(YELLOW);
  tft.setCursor(45, 80);
  tft.print("IDLE");
}

void drawSpeedPanel() {
  tft.fillRoundRect(10, 150, SCREEN_W - 20, 55, 8, DARKGREY);
  tft.setTextColor(CYAN);
  tft.setTextSize(1);
  tft.setCursor(20, 158);
  tft.print("SPEED  (Joystick Y)");
  tft.setCursor(20, 170);
  tft.print("Value:");
  tft.setCursor(130, 170);
  tft.print("0%");
  tft.fillRect(20, 183, 200, 12, BLACK);
  tft.drawRect(20, 183, 200, 12, WHITE);
}

void drawButtonPanel() {
  tft.fillRoundRect(10, 215, SCREEN_W - 20, 100, 8, DARKGREY);
  tft.setTextColor(CYAN);
  tft.setTextSize(1);
  tft.setCursor(20, 223);
  tft.print("DIRECTION BUTTONS");
  tft.setTextColor(WHITE);
  tft.setCursor(22, 240);  tft.print("FWD");
  tft.setCursor(78, 240);  tft.print("BACK");
  tft.setCursor(143, 240); tft.print("LEFT");
  tft.setCursor(195, 240); tft.print("RGHT");
  tft.drawRoundRect(15,  252, 48, 48, 6, WHITE);
  tft.drawRoundRect(68,  252, 52, 48, 6, WHITE);
  tft.drawRoundRect(135, 252, 48, 48, 6, WHITE);
  tft.drawRoundRect(185, 252, 42, 48, 6, WHITE);
}

void drawStatusBar() {
  tft.fillRect(0, 310, SCREEN_W, 10, DARKGREY);
  tft.setTextColor(RED);
  tft.setTextSize(1);
  tft.setCursor(5, 311);
  tft.print("BT: ---");
}

// ==============================
// Update Functions
// ==============================
String getDirection(bool fwd, bool back, bool left, bool right) {
  if (fwd)   return "FWD ";
  if (back)  return "BACK";
  if (left)  return "LEFT";
  if (right) return "RGHT";
  return "IDLE";
}

int getSpeed(int y) {
  int center = 2048;
  int diff = abs(y - center);
  return constrain(map(diff, 0, 2048, 0, 100), 0, 100);
}

void updateDirection(String dir) {
  if (dir != prevDirection) {
    tft.fillRoundRect(10, 55, SCREEN_W - 20, 85, 8, DARKGREY);
    tft.setTextColor(CYAN);
    tft.setTextSize(1);
    tft.setCursor(20, 63);
    tft.print("DIRECTION");
    uint16_t dirColour = YELLOW;
    if (dir == "FWD ")  dirColour = GREEN;
    if (dir == "BACK")  dirColour = RED;
    if (dir == "LEFT")  dirColour = CYAN;
    if (dir == "RGHT")  dirColour = ORANGE;
    if (dir == "IDLE")  dirColour = YELLOW;
    tft.setTextSize(4);
    tft.setTextColor(dirColour);
    tft.setCursor(45, 80);
    tft.print(dir);
    prevDirection = dir;
  }
}

void updateSpeed(int spd) {
  if (spd != prevSpeed) {
    tft.fillRect(65, 170, 60, 10, DARKGREY);
    tft.setTextColor(WHITE);
    tft.setTextSize(1);
    tft.setCursor(65, 170);
    tft.print(analogRead(JOY_Y));
    tft.fillRect(130, 170, 50, 10, DARKGREY);
    tft.setTextColor(WHITE);
    tft.setCursor(130, 170);
    tft.print(spd);
    tft.print("%");
    tft.fillRect(21, 184, 198, 10, BLACK);
    uint16_t barColour = GREEN;
    if (spd > 66)      barColour = RED;
    else if (spd > 33) barColour = ORANGE;
    int barWidth = map(spd, 0, 100, 0, 198);
    tft.fillRect(21, 184, barWidth, 10, barColour);
    prevSpeed = spd;
  }
}

void updateButtons(bool btns[4]) {
  int xPos[4]   = {15, 68, 135, 185};
  int bWidth[4] = {48, 52, 48, 42};
  String labels[4]       = {"FWD", "BACK", "LEFT", "RGHT"};
  uint16_t onColour[4]   = {GREEN, RED, CYAN, ORANGE};
  for (int i = 0; i < 4; i++) {
    if (btns[i] != prevBtns[i]) {
      tft.fillRoundRect(xPos[i], 252, bWidth[i], 48, 6,
                        btns[i] ? onColour[i] : DARKGREY);
      tft.drawRoundRect(xPos[i], 252, bWidth[i], 48, 6, WHITE);
      tft.setTextSize(1);
      tft.setTextColor(btns[i] ? BLACK : WHITE);
      tft.setCursor(xPos[i] + 8, 272);
      tft.print(labels[i]);
      prevBtns[i] = btns[i];
    }
  }
}

void updateStatusBar(String dir, int spd) {
  tft.fillRect(0, 310, SCREEN_W, 10, DARKGREY);
  tft.setTextColor(deviceConnected ? GREEN : RED);
  tft.setTextSize(1);
  tft.setCursor(5, 311);
  tft.print(deviceConnected ? "BT: OK  " : "BT: --- ");
  tft.setTextColor(WHITE);
  tft.print("DIR:");
  tft.print(dir);
  tft.print(" SPD:");
  tft.print(spd);
  tft.print("%");
}

// ==============================
// Setup
// ==============================
void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(BUZZER, OUTPUT);
  digitalWrite(BUZZER, LOW);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  pinMode(JOY_Y, INPUT);

  pinMode(JOY_BTN,   INPUT_PULLUP);
  pinMode(BTN_FWD,   INPUT_PULLUP);
  pinMode(BTN_BACK,  INPUT_PULLUP);
  pinMode(BTN_LEFT,  INPUT_PULLUP);
  pinMode(BTN_RIGHT, INPUT_PULLUP);

  tft.begin();
  tft.setRotation(2);
  tft.fillScreen(BLACK);

  drawHeader();
  drawDirectionPanel();
  drawSpeedPanel();
  drawButtonPanel();
  drawStatusBar();

  setupBLE();
  startupBeep();
  setupAudio();         // Add this line
  startupBeep();
  

  Serial.println("Setup complete");
}

// ==============================
// Loop
// ==============================
void loop() {
  int jy  = analogRead(JOY_Y);
  int spd = getSpeed(jy);

  bool btns[4] = {
    !digitalRead(BTN_FWD),
    !digitalRead(BTN_BACK),
    !digitalRead(BTN_LEFT),
    !digitalRead(BTN_RIGHT)
  };

  // Detect joystick button press for audio beep
  static bool lastJoyBtn = false;
  bool joyBtn = !digitalRead(JOY_BTN);
  if (joyBtn && !lastJoyBtn) {
    beepAudio(80);
  }
  lastJoyBtn = joyBtn;

  String dir = getDirection(btns[1], btns[3], btns[0], btns[2]);
  String cmd = dir + ":" + String(spd);

  // Only send if command changed
  if (cmd != lastCmd) {
    sendCommand(dir, spd);
    lastCmd = cmd;
  }

  updateDirection(dir);
  updateSpeed(spd);
  updateButtons(btns);
  updateStatusBar(dir, spd);

  Serial.printf("DIR: %s  SPD: %d%%  JY: %d  BT: %s\n",
    dir.c_str(), spd, jy, deviceConnected ? "OK" : "---");

  // Handle BLE reconnection
  if (!deviceConnected && oldDeviceConnected) {
    delay(500);
    BLEDevice::startAdvertising();
    oldDeviceConnected = deviceConnected;
  }
  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
  }

  delay(50);
}