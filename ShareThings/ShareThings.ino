#include <Wire.h>
#define SDA_PIN 21
#define SCL_PIN 22


void setup() {
  while (!Serial);

  Serial.begin(9600);
  Serial.println("started");

  Wire.begin(SDA_PIN, SCL_PIN);
}

//メイン関数(動作は繰り返す)
void loop() {
  //  i2c_scanner();
  cmd_kaiten_lamp(100, 1);
  cmd_servo(1, 0, 90);
  cmd_servo(2, 0, 45);
  delay(5000);
  cmd_kaiten_lamp(0, 0);
  cmd_servo(1, 180, 90);
  cmd_servo(2, 90, 45);
  delay(5000);
}

static const int kaiten_addr = 0x15;

void cmd_kaiten(uint8_t duty) {
  char kaiten_cmd[2];

  kaiten_cmd[0] = 0x01;
  kaiten_cmd[1] = duty;

  Wire.beginTransmission(kaiten_addr);
  for (int i = 0; i < sizeof(kaiten_cmd) / sizeof(kaiten_cmd[0]); i++) {
    Wire.write(kaiten_cmd[i]);
  }
  Wire.endTransmission();
}

void cmd_lamp(bool lamp) {
  char kaiten_cmd[2];

  kaiten_cmd[0] = 0x02;
  kaiten_cmd[1] = lamp;

  Wire.beginTransmission(kaiten_addr);
  for (int i = 0; i < sizeof(kaiten_cmd) / sizeof(kaiten_cmd[0]); i++) {
    Wire.write(kaiten_cmd[i]);
  }
  Wire.endTransmission();
}

void cmd_kaiten_lamp(uint8_t duty, bool lamp) {
  cmd_kaiten(duty);
  cmd_lamp(lamp);
}

static const int servo_addr = 0x25;

void cmd_servo(uint8_t servo_no, uint8_t angle, uint8_t speed) {
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


void i2c_scanner()
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
