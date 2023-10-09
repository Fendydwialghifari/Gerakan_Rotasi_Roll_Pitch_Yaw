/*pin aesp8266 to sd card
  gnd = gnd
  vcc = vv
  miso = D6
  mosi = D7
  sck = D5
  cs = D8

  To mpu9250
  gnd = gnd
  vcc = 3 v
  scl = D1
  sda = D2

  OLED
  gnd = gnd
  vcc = 3.3
  sck = D1
  SDA = D2

  LED
  HIJAU = D4
  MERAH = D3
*/
#include <Wire.h>
#include <MPU9250.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

MPU9250 IMU(Wire, 0x68); // Change address to 0x69 if necessary
int status;

#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels
#define OLED_RESET    -1  // Reset pin # (or -1 if sharing reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

const int chipSelect = D8;

unsigned long previousMillis = 0;
const long intervaltime = 50;

float roll, pitch, yaw;

float Kal_Roll, Kal_Pitch, Kal_Yaw;

float Xt_Roll, Xt_update_Roll, Xt_prev_Roll;
float Xt_Pitch, Xt_update_Pitch, Xt_prev_Pitch;
float Xt_Yaw, Xt_update_Yaw, Xt_prev_Yaw;

float Pt, Pt_update, Pt_prev;
float Kt, R, Q;

void setup() {
  Serial.begin(115200);
  R = 1; Q = 0.1; Pt_prev = 1;
  pinMode(D3, OUTPUT);
  pinMode(D4, OUTPUT);
  while (!Serial) {}

  // start communication with IMU
  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while (1) {}
  }
  Serial.print("Initializing SD card...");

  //pesan status SDcard
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    while (1);
  }
  Serial.println("card initialized.");

  // pesan status oled display
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
  {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;); // Don't proceed, loop forever
  }
  display.display();
  delay(2);
  display.clearDisplay();

  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.print("MPU Angles");
  display.display();
  delay(1000);
}
void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= intervaltime)
  {
    float time_stamp = currentMillis / 1000.00;
    previousMillis = currentMillis;
    IMU.readSensor();
    float AccX = IMU.getAccelX_mss();
    float AccY = IMU.getAccelY_mss();
    float AccZ = IMU.getAccelZ_mss();
    float GyroZ = IMU.getGyroZ_rads();
    
    float pitch = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * 1 / (3.142 / 180);
    float roll = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * 1 / (3.142 / 180);

    //Yaw from mag

    float heading = atan2(IMU.getMagY_uT(), IMU.getMagX_uT());
      float declinationAngle = (4.0 + (26.0 / 60.0)) / (180 / PI);
      heading += declinationAngle;
      // Correct for heading < 0deg and heading > 360deg
      if (heading < 0) {
      heading += 2 * PI;
      }
      if (heading > 2 * PI) {
      heading -= 2 * PI;
      }
    // Calculate yaw angle by integrating gyroscope data
    //yaw += GyroZ * 0.05; // 0.05 is the time interval (dt) in seconds

    // Convert to degrees
    float yaw = heading * 180 / M_PI;

    Xt_update_Roll = Xt_prev_Roll;
    Xt_update_Pitch = Xt_prev_Pitch;
    Xt_update_Yaw = Xt_prev_Yaw;

    Pt_update = Pt_prev + Q;
    Kt = Pt_update / (Pt_update + R);

    Xt_Roll = Xt_update_Roll + ( Kt * (roll - Xt_update_Roll));
    Xt_Pitch = Xt_update_Pitch + ( Kt * (pitch - Xt_update_Pitch));
    Xt_Yaw = Xt_update_Yaw + ( Kt * (yaw - Xt_update_Yaw));

    Pt = (1 - Kt) * Pt_update;

    Xt_prev_Roll = Xt_Roll;
    Xt_prev_Pitch = Xt_Pitch;
    Xt_prev_Yaw = Xt_Yaw;

    Pt_prev = Pt;

    Kal_Roll = Xt_Roll;
    Kal_Pitch = Xt_Pitch;
    Kal_Yaw = Xt_Yaw;

    Serial.print(time_stamp);
    Serial.print(",");
    Serial.print(roll);
    Serial.print(",");
    Serial.print(Kal_Roll);
    Serial.print(",");
    Serial.print(pitch);
    Serial.print(",");
    Serial.print(Kal_Pitch);
    Serial.print(",");
    Serial.print(yaw);
    Serial.print(",");
    Serial.print(Kal_Yaw);
    Serial.println();

    File dataFile = SD.open("datalog.txt", FILE_WRITE);
    if (dataFile) {
      digitalWrite(D3, HIGH);
      digitalWrite(D4, LOW);
      dataFile.print(time_stamp);
      dataFile.print(",");
      dataFile.print(roll);
      dataFile.print(",");
      dataFile.print(Kal_Roll);
      dataFile.print(",");
      dataFile.print(pitch);
      dataFile.print(",");
      dataFile.print(Kal_Pitch);
      dataFile.print(",");
      dataFile.print(yaw);
      dataFile.print(",");
      dataFile.println(Kal_Yaw);
      dataFile.close();
    }
    else {
      Serial.println("error opening datalog.txt");
      digitalWrite(D4, HIGH);
      digitalWrite(D3, LOW);
    }

    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(25, 0);
    display.print("MPU9250");

    display.setTextSize(2);
    display.setCursor(0, 16);
    display.print("R: ");
    display.println(Kal_Roll);

    display.setTextSize(2);
    display.setCursor(0, 33);
    display.print("P: ");
    display.println(Kal_Pitch);

    display.setTextSize(2);
    display.setCursor(0, 50);
    display.print("Y: ");
    display.println(Kal_Yaw);
    display.display();
  }
}
