/*
         [ UPDATE ]       [ AUTHOR ]
        =2021.07.01=     =Sean Chang=
*/
#include <math.h>
#include <SimpleTimer.h>
#include <HardwareSerial.h>
#include <BluetoothSerial.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
BluetoothSerial SerialBT;

#include <MPU9250.h>

MPU9250 IMU(Wire, 0x68);
int status;

SimpleTimer timer;
HardwareSerial UWBSerial(1);

//========================================//
int timerId;

float x1 = 0.00, y_1 = 0.00,
      x2 = 142.00, y2 = 0.00,
      x3 = 0.00, y3 = 230.00;
float x = 0.00 , y = 0.00;
float delta = 85; //CM
String UWB_Data;
String an1, an2, an3;

char BT;
//========================================//

//============= MOTOR CONTROL ============//
//ENA
const int IN1 = 27;
const int IN2 = 26;
const int ENA = 14;
//ENB
const int IN3 = 25;
const int IN4 = 33;
const int ENB = 32;

// Setting PWM properties
const int freq = 30000;
const int pwmChannel_A = 1;
const int pwmChannel_B = 2;
const int resolution = 8;
int speed_L = 0;
int speed_R = 0;
//========================================//

//================ IMU ===================//
float gyroX, gyroX_degree;
float gyroXerror = 0.03;
float range = 0.05;
unsigned long gyroDelay = 100;
unsigned long lastTime = 0;
//========================================//
float AIM_X = 130.00;
float AIM_Y = 200.00;
float theta, theta_1, theta_2;

TaskHandle_t Task1;



void setup() {
  Serial.begin(115200);
  UWBSerial.begin(115200, SERIAL_8N1, 16, 17);
  SerialBT.begin("ESP32-UWB"); //Bluetooth device name

  while (!Serial) {}
  Serial.println("\n= NodeMCU ESP32 DONE =");

  UWBSerial.println("AT");
  while (!UWBSerial.available()) {
    delay(200);
    Serial.println("= UWB BU01 Detecting =");
  }

  timerId = timer.setInterval(1283L, sendDATA);
  timer.disable(timerId);

  //============= MOTOR CONTROL ============//
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  // configure LED PWM functionalitites
  ledcSetup(pwmChannel_A, freq, resolution);
  ledcSetup(pwmChannel_B, freq, resolution);

  // attach the channel to the GPIO to be controlled
  ledcAttachPin(ENA, pwmChannel_A);
  ledcAttachPin(ENB, pwmChannel_B);
  //========================================//

  //================ IMU ===================//
  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while (1) {}
  }
  IMU.setAccelRange(MPU9250::ACCEL_RANGE_8G);
  IMU.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
  IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
  IMU.setSrd(19);
  //========================================//
  xTaskCreatePinnedToCore(Task1_readIMU, "Task1", 10000, NULL, 0, &Task1, 0);
}

void loop() {
  serialBridge();
  AN_detect();

  float an1_f = an1.toFloat();
  float an2_f = an2.toFloat();
  float an3_f = an3.toFloat();
  an1_f = an1_f * 100;
  an2_f = an2_f * 100;
  an3_f = an3_f * 100;
  an1_f = sqrt(an1_f * an1_f - delta * delta);
  an2_f = sqrt(an2_f * an2_f - delta * delta);
  an3_f = sqrt(an3_f * an3_f - delta * delta);
  float a = (-2 * x1 + 2 * x2);
  float b = (-2 * y_1 + 2 * y2);
  float c = (an1_f * an1_f - an2_f * an2_f - x1 * x1 + x2 * x2 - y_1 * y_1 + y2 * y2);
  float d = (-2 * x2 + 2 * x3);
  float e = (-2 * y2 + 2 * y3);
  float f = (an2_f * an2_f - an3_f * an3_f - x2 * x2 + x3 * x3 - y2 * y2 + y3 * y3);
  x = (c * e - f * b) / (e * a - b * d);
  y = (c * d - a * f) / (b * d - a * e);

  //readIMU();
  ACTION();
  MOTOR_CONTROL();

  if (timer.isEnabled(timerId)) {
    timer.run();
  }

  UWB_Data = ""; an1 = ""; an2 = ""; an3 = "";
}


void ACTION() {
  float delta_X, delta_Y;
  delta_X = AIM_X - x;
  delta_Y = AIM_Y - y;
  theta = atan2(delta_Y , delta_X);
}

void MOTOR_CONTROL () {

  //digitalWrite(IN1, HIGH);
  //digitalWrite(IN2, LOW);
  //digitalWrite(IN3, HIGH);
  //digitalWrite(IN4, LOW);
  ledcWrite(pwmChannel_A, speed_L);
  ledcWrite(pwmChannel_B, speed_R);

  if (theta > 0) {
    theta_1 = 1.57 - theta;
  } else if (theta < 0) {
    theta_2 = -1.57 - theta;
  }


  if (x < 135 && x > 125) {
    if (y < 205 && y > 195) {
      speed_L = 0;
      speed_R = 0;
      SerialBT.println("END STOP");
    }
  }
  else {
    if (theta_1 > gyroX) {  //Turn Right
      speed_L = 175;
      speed_R = 165;
    }
    else if (theta_2 < gyroX) {  //Turn Left
      speed_R = 175;
      speed_L = 165;
    }
    /*
      if (theta_1 > gyroX && gyroX < 1.57) {  //Turn Right
      speed_L = 175;
      speed_R = 165;
      } else if (gyroX > 1.57) {  //Turn Left
      speed_R = 175;
      speed_L = 165;
      }
      if (theta_2 < gyroX && gyroX > -1.57) {  //Turn Left
      speed_R = 175;
      speed_L = 165;
      } else if (gyroX < -1.57) {  //Turn Right
      speed_L = 175;
      speed_R = 165;
      }
    */
  }
  //Serial.print(gyroX, 4);
  //Serial.print("  |  ");
  //Serial.println(gyroX_degree, 4);
}

/*
  void readIMU () {
  IMU.readSensor();
  if ((millis() - lastTime) > gyroDelay) {
    float gyroX_temp = IMU.getGyroX_rads();
    if (abs(gyroX_temp) > gyroXerror)  {
      //gyroX += gyroX_temp * 0.05 / (4.25 * 1.06);
      //gyroX += gyroX_temp * 0.05*1.266 / (4.25 * 1.06);
      gyroX += gyroX_temp * 0.014;
      gyroX_degree = gyroX * 57.2958;
    }
    lastTime = millis();
  }
  //SerialBT.println(gyroX);
  }
*/
void Task1_readIMU(void * pvParameters ) {
  for (;;) {
    IMU.readSensor();
    if ((millis() - lastTime) > gyroDelay) {
      float gyroX_temp = IMU.getGyroX_rads();
      if (abs(gyroX_temp) > gyroXerror)  {
        gyroX += gyroX_temp * 9.2 * 0.016 / 1.445;
        gyroX_degree = gyroX * 57.2958;
      }
      lastTime = millis();
    }
    //SerialBT.println(gyroX);
  }
}

void serialTask() {
  if (BT == '9') {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    speed_L = 150;
    speed_R = 150;
    SerialBT.println("Tracking AIM");
  }
  if (BT == '8') {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    speed_L = 150;
    speed_R = 150;
    SerialBT.println("Stop Tracking");
  }
  if (BT == '7') {
    gyroX = 0;
    SerialBT.println("IMU Reset");
    SerialBT.print(gyroX);
    SerialBT.print(" | ");
    SerialBT.println(gyroX_degree);
  }
}

void serialBridge () {
  if (SerialBT.available()) {
    BT = SerialBT.read();
    UWBSerial.write(BT);
    serialTask();
  }
  if (Serial.available()) {
    UWBSerial.write(Serial.read());
  }
  if (UWBSerial.available()) {
    UWB_Data = UWBSerial.readString();
    Serial.print(UWB_Data);
    SerialBT.print(UWB_Data);
  }
}

void sendDATA () {
  Serial.println("an1=" + an1 + "," + "an2=" + an2 + "," + "an3=" + an3);
  Serial.print("x=");
  Serial.print(x, 2);
  Serial.print(",");
  Serial.print("y=");
  Serial.println(y, 2);

  SerialBT.print("x=");
  SerialBT.print(x, 2);
  SerialBT.print(",");
  SerialBT.print("y=");
  SerialBT.print(y, 2);
  SerialBT.print(",");
  SerialBT.print("r=");
  SerialBT.println(gyroX_degree, 2);
}

void AN_detect () {
  if (UWB_Data.substring(0, 3) == "an1") {
    if (UWB_Data.substring(11, 14) == "an2") {
      if (UWB_Data.substring(22, 25) == "an3") {
        an1 = UWB_Data.substring(4, 8);
        an2 = UWB_Data.substring(15, 19);
        an3 = UWB_Data.substring(26, 30);
      } else {
        an1 = UWB_Data.substring(4, 8);
        an2 = UWB_Data.substring(15, 19);
      }
    } else {
      if (UWB_Data.substring(11, 14) == "an3") {
        an1 = UWB_Data.substring(4, 8);
        an3 = UWB_Data.substring(15, 19);
      } else {
        an1 = UWB_Data.substring(4, 8);
      }
    }
    timer.enable(timerId);
  } else if (UWB_Data.substring(0, 3) == "an2") {
    if (UWB_Data.substring(11, 14) == "an3") {
      an2 = UWB_Data.substring(4, 8);
      an3 = UWB_Data.substring(15, 19);
    } else {
      an2 = UWB_Data.substring(4, 8);
    }
    timer.enable(timerId);
  } else if (UWB_Data.substring(0, 3) == "an3") {
    an3 = UWB_Data.substring(4, 8);
    timer.enable(timerId);
  } else {
    timer.disable(timerId);
  }
}
