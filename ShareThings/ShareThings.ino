#include <BLEServer.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Wire.h>

#define SDA_PIN 21
#define SCL_PIN 22

// Device Name: Maximum 30 bytes
#define DEVICE_NAME "LINE Things Trial ESP32"

// User service UUID: Change this to your generated service UUID
#define USER_SERVICE_UUID "3fd1e37b-83a3-4691-8a70-dc42cd486ef7"
// User service characteristics
#define WRITE_CHARACTERISTIC_UUID "E9062E71-9E62-4BC6-B0D3-35CDCD9B027B"
#define NOTIFY_CHARACTERISTIC_UUID "62FBD229-6EDD-4D1A-B554-5C4E1BB29169"

// PSDI Service UUID: Fixed value for Developer Trial
#define PSDI_SERVICE_UUID "E625601E-9E55-4597-A598-76018A0D293D"
#define PSDI_CHARACTERISTIC_UUID "26E2B12B-85F0-4F3F-9FDD-91D114270E6E"

#define BUTTON 0
#define LED1 2

BLEServer* thingsServer;
BLESecurity *thingsSecurity;
BLEService* userService;
BLEService* psdiService;
BLECharacteristic* psdiCharacteristic;
BLECharacteristic* writeCharacteristic;
BLECharacteristic* notifyCharacteristic;

bool deviceConnected = false;
bool oldDeviceConnected = false;

volatile int btnAction = 0;


void action1();
void action2();
static void cmd_kaiten_lamp(uint8_t duty, bool lamp);
static void cmd_servo(uint8_t servo_no, uint8_t angle, uint8_t speed);

class serverCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

#define BOX1_LOCK     0x00
#define BOX2_LOCK     0x01
#define WARNING_LIGHT 0x02
#define TEST_LED      0xff

#define LOCK    1
#define UNLOCK  0

#define LOCKED_ANGLE      20
#define UNLOCKED_ANGLE    90

#define ERROR_DETECT_PIN  5

class writeCallback: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *bleWriteCharacteristic) {
      std::string value = bleWriteCharacteristic->getValue();
      switch ((char)value[0]) {
        case BOX1_LOCK: {
            char lock_state = (char)value[1];
            if (lock_state == UNLOCK) {
              cmd_servo(1, UNLOCKED_ANGLE, 90);
            } else {
              cmd_servo(1, LOCKED_ANGLE, 90);
            }
            break;
          }
        case BOX2_LOCK: {
            char lock_state = (char)value[1];
            if (lock_state == UNLOCK) {
              cmd_servo(2, UNLOCKED_ANGLE, 90);
            } else {
              cmd_servo(2, LOCKED_ANGLE, 90);
            }
            break;
          }
        case WARNING_LIGHT: {
            char duty = (char)value[1];
            if (duty == 0) {
              //              cmd_kaiten_lamp(0, 0);
            } else {
              //              cmd_kaiten_lamp(duty, 1);
            }
            break;
          }
        case TEST_LED: {
            char led_state = (char)value[1];
            if (led_state == 0) {
              digitalWrite(LED1, LOW);
            } else {
              digitalWrite(LED1, HIGH);
            }
            break;
          }
      }
    }
};

void setup() {
  Serial.begin(115200);

  device_init();

  pinMode(ERROR_DETECT_PIN, INPUT);

  pinMode(LED1, OUTPUT);
  pinMode(BUTTON, INPUT_PULLUP);
  attachInterrupt(BUTTON, buttonAction, CHANGE);

  BLEDevice::init("");
  BLEDevice::setEncryptionLevel(ESP_BLE_SEC_ENCRYPT_NO_MITM);

  // Security Settings
  BLESecurity *thingsSecurity = new BLESecurity();
  thingsSecurity->setAuthenticationMode(ESP_LE_AUTH_REQ_SC_ONLY);
  thingsSecurity->setCapability(ESP_IO_CAP_NONE);
  thingsSecurity->setInitEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);

  setupServices();
  startAdvertising();
  Serial.println("Ready to Connect");
}

static void error_proc(void)
{
  bool error;
  static bool error_old;
  if (digitalRead(ERROR_DETECT_PIN) == LOW) {
    error = false;
  } else {
    error = true;
  }
  if (error_old != error) {
    Serial.print("Error: ");
    Serial.println(error);
    if (error == true) {
      cmd_kaiten_lamp(100, 1);
    } else {
      cmd_kaiten_lamp(0, 0);
    }
  }
  error_old = error;
}

void loop() {
  uint8_t btnValue;
  error_proc();

  while (btnAction > 0 && deviceConnected) {
    btnValue = !digitalRead(BUTTON);
    btnAction = 0;
    notifyCharacteristic->setValue(&btnValue, 1);
    notifyCharacteristic->notify();
    delay(20);
  }
  // Disconnection
  if (!deviceConnected && oldDeviceConnected) {
    delay(500); // Wait for BLE Stack to be ready
    thingsServer->startAdvertising(); // Restart advertising
    oldDeviceConnected = deviceConnected;
  }
  // Connection
  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
  }
}

void setupServices(void) {
  // Create BLE Server
  thingsServer = BLEDevice::createServer();
  thingsServer->setCallbacks(new serverCallbacks());

  // Setup User Service
  userService = thingsServer->createService(USER_SERVICE_UUID);
  // Create Characteristics for User Service
  writeCharacteristic = userService->createCharacteristic(WRITE_CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_WRITE);
  writeCharacteristic->setAccessPermissions(ESP_GATT_PERM_READ_ENCRYPTED | ESP_GATT_PERM_WRITE_ENCRYPTED);
  writeCharacteristic->setCallbacks(new writeCallback());

  notifyCharacteristic = userService->createCharacteristic(NOTIFY_CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  notifyCharacteristic->setAccessPermissions(ESP_GATT_PERM_READ_ENCRYPTED | ESP_GATT_PERM_WRITE_ENCRYPTED);
  BLE2902* ble9202 = new BLE2902();
  ble9202->setNotifications(true);
  ble9202->setAccessPermissions(ESP_GATT_PERM_READ_ENCRYPTED | ESP_GATT_PERM_WRITE_ENCRYPTED);
  notifyCharacteristic->addDescriptor(ble9202);

  // Setup PSDI Service
  psdiService = thingsServer->createService(PSDI_SERVICE_UUID);
  psdiCharacteristic = psdiService->createCharacteristic(PSDI_CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_READ);
  psdiCharacteristic->setAccessPermissions(ESP_GATT_PERM_READ_ENCRYPTED | ESP_GATT_PERM_WRITE_ENCRYPTED);

  // Set PSDI (Product Specific Device ID) value
  uint64_t macAddress = ESP.getEfuseMac();
  psdiCharacteristic->setValue((uint8_t*) &macAddress, sizeof(macAddress));

  // Start BLE Services
  userService->start();
  psdiService->start();
}

void startAdvertising(void) {
  // Start Advertising
  BLEAdvertisementData scanResponseData = BLEAdvertisementData();
  scanResponseData.setFlags(0x06); // GENERAL_DISC_MODE 0x02 | BR_EDR_NOT_SUPPORTED 0x04
  scanResponseData.setName(DEVICE_NAME);

  thingsServer->getAdvertising()->addServiceUUID(userService->getUUID());
  thingsServer->getAdvertising()->setScanResponseData(scanResponseData);
  thingsServer->getAdvertising()->start();
}

void buttonAction() {
  btnAction++;
}




void device_init() {
  Wire.begin(SDA_PIN, SCL_PIN);
}


void action1() {
  cmd_kaiten_lamp(100, 1);
  cmd_servo(1, 0, 90);
  cmd_servo(2, 0, 45);
}

void action2() {
  cmd_kaiten_lamp(0, 0);
  cmd_servo(1, 180, 90);
  cmd_servo(2, 90, 45);
}

static const int kaiten_addr = 0x15;

static void cmd_kaiten(uint8_t duty) {
  char kaiten_cmd[2];

  kaiten_cmd[0] = 0x01;
  kaiten_cmd[1] = duty;

  Wire.beginTransmission(kaiten_addr);
  for (int i = 0; i < sizeof(kaiten_cmd) / sizeof(kaiten_cmd[0]); i++) {
    Wire.write(kaiten_cmd[i]);
  }
  Wire.endTransmission();
}

static void cmd_lamp(bool lamp) {
  char kaiten_cmd[2];

  kaiten_cmd[0] = 0x02;
  kaiten_cmd[1] = lamp;

  Wire.beginTransmission(kaiten_addr);
  for (int i = 0; i < sizeof(kaiten_cmd) / sizeof(kaiten_cmd[0]); i++) {
    Wire.write(kaiten_cmd[i]);
  }
  Wire.endTransmission();
}

static void cmd_kaiten_lamp(uint8_t duty, bool lamp) {
  cmd_kaiten(duty);
  cmd_lamp(lamp);
}

static const int servo_addr = 0x25;

static void cmd_servo(uint8_t servo_no, uint8_t angle, uint8_t speed) {
  char servo_cmd[4] = { 0x01, 0x01, 0, 90 };
  servo_cmd[0] = 0x01;
  servo_cmd[1] = servo_no;
  servo_cmd[2] = angle;
  servo_cmd[3] = speed;
  Wire.beginTransmission(servo_addr);
  for (int i = 0; i < sizeof(servo_cmd) / sizeof(servo_cmd[0]); i++) {
    Wire.write(servo_cmd[i]);
  }
  Wire.endTransmission();
}


static void i2c_scanner()
{
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for (address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error == 4)
    {
      Serial.print("Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");

  delay(5000);           // wait 5 seconds for next scan
}

