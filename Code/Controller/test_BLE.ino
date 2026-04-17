#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEClient.h>
#include <BLEAdvertisedDevice.h>

// Must match controller exactly
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CONTROLLER_NAME     "ESP32-Controller"

BLEClient                  *pClient     = nullptr;
BLERemoteCharacteristic    *pRemoteChar = nullptr;
BLEAdvertisedDevice        *myDevice    = nullptr;

bool doConnect  = false;
bool connected  = false;
bool doScan     = false;

// ==============================
// Notification Callback
// Fires every time controller sends data
// ==============================
static void notifyCallback(
  BLERemoteCharacteristic *pBLERemoteCharacteristic,
  uint8_t *pData,
  size_t length,
  bool isNotify)
{
  String cmd = "";
  for (int i = 0; i < length; i++) {
    cmd += (char)pData[i];
  }

  int colonIdx = cmd.indexOf(':');
  if (colonIdx < 0) {
    Serial.printf("Raw RX: %s\n", cmd.c_str());
    return;
  }

  String dir     = cmd.substring(0, colonIdx);
  int    dutyVal = cmd.substring(colonIdx + 1).toInt();
  int    spdPct  = map(dutyVal, 0, 1023, 0, 100);

  Serial.println("─────────────────────────────");
  Serial.printf("RAW:    %s\n",  cmd.c_str());
  Serial.printf("DIR:    %s\n",  dir.c_str());
  Serial.printf("DUTY:   %d\n",  dutyVal);
  Serial.printf("SPEED:  %d%%\n", spdPct);

  // What the motor code would do
  if      (dir == "FWD ")  Serial.printf("MOTOR: move_forward(%d)\n",  dutyVal);
  else if (dir == "BACK")  Serial.printf("MOTOR: move_reverse(%d)\n",  dutyVal);
  else if (dir == "LEFT")  Serial.printf("MOTOR: move_left(%d)\n",     dutyVal);
  else if (dir == "RGHT")  Serial.printf("MOTOR: move_right(%d)\n",    dutyVal);
  else                     Serial.println("MOTOR: stop_all()");
}

// ==============================
// Scan Callback
// ==============================
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    Serial.printf("Found: %s\n", advertisedDevice.getName().c_str());
    if (advertisedDevice.getName() == CONTROLLER_NAME) {
      Serial.println("Target found — stopping scan");
      BLEDevice::getScan()->stop();
      myDevice  = new BLEAdvertisedDevice(advertisedDevice);
      doConnect = true;
      doScan    = false;
    }
  }
};

// ==============================
// Connect
// ==============================
bool connectToController() {
  Serial.printf("Connecting to %s ...\n",
    myDevice->getAddress().toString().c_str());

  pClient = BLEDevice::createClient();

  if (!pClient->connect(myDevice)) {
    Serial.println("Connection failed");
    return false;
  }
  Serial.println("Connected");

  BLERemoteService *pService = pClient->getService(SERVICE_UUID);
  if (!pService) {
    Serial.println("Service not found");
    pClient->disconnect();
    return false;
  }
  Serial.println("Service found");

  pRemoteChar = pService->getCharacteristic(CHARACTERISTIC_UUID);
  if (!pRemoteChar) {
    Serial.println("Characteristic not found");
    pClient->disconnect();
    return false;
  }
  Serial.println("Characteristic found");

  if (pRemoteChar->canNotify()) {
    pRemoteChar->registerForNotify(notifyCallback);
    Serial.println("Subscribed to notifications");
  } else {
    Serial.println("Notifications not supported");
    return false;
  }

  connected = true;
  return true;
}

// ==============================
// Setup
// ==============================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\n=== BLE Motor Command Test ===");
  Serial.println("Scanning for ESP32-Controller...\n");

  BLEDevice::init("ESP32-Vehicle-Test");

  BLEScan *pScan = BLEDevice::getScan();
  pScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pScan->setInterval(1349);
  pScan->setWindow(449);
  pScan->setActiveScan(true);
  pScan->start(30, false);
}

// ==============================
// Loop
// ==============================
void loop() {
  if (doConnect) {
    if (connectToController()) {
      Serial.println("\nReady — press buttons on controller\n");
    } else {
      Serial.println("Connect failed — will rescan");
      doScan = true;
    }
    doConnect = false;
  }

  if (!connected && doScan) {
    Serial.println("Rescanning...");
    BLEDevice::getScan()->start(5, false);
    doScan = false;
  }

  if (connected && !pClient->isConnected()) {
    connected = false;
    doScan    = true;
    Serial.println("Lost connection — rescanning...");
  }

  delay(100);
}