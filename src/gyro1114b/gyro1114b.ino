#include <PID_v1.h>
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Servo.h"
Servo myservox;
Servo myservoy;
int angley=88;
int anglex=70;
int angleylow=angley-3;
int angleyhigh=angley+3;
int anglexlow=anglex-2;
int anglexhigh=anglex+2;
int ServoOutputx,ServoOutputy;
void setup() {
    myservoy.attach(10);  
    myservoy.write(70);
    myservox.attach(9);
    myservox.write(88);
  // Serial init
  Serial.begin(9600);
  // Wire init
  Wire.begin();
  // Power Management
  Wire.beginTransmission(0x68);
  Wire.write(107);
  Wire.write(0);
  Wire.endTransmission();
  // Register 26
  for(uint8_t i = 2; i <= 7; i++)
  {
    Wire.beginTransmission(0x68);
    Wire.write(26);
    Wire.write(i << 3 | 0x03);
    Wire.endTransmission();
  }
  // Register 27
  Wire.beginTransmission(0x68);
  Wire.write(27);
  Wire.write(3 << 3);
  Wire.endTransmission();
  // Register 28
  Wire.beginTransmission(0x68);
  Wire.write(28);
  Wire.write(0);
  Wire.endTransmission();
}
int16_t offset[3] = {-22, 15, -4};

void loop() {
  uint8_t i;
  static int16_t acc_raw[3]={0,}, gyro_raw[3]={0,};
  // Get Accel
  Wire.beginTransmission(0x68);
  Wire.write(59);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  for(i = 0; i < 3; i++) acc_raw[i] = (Wire.read() << 8) | Wire.read();
  // Get Gyro
  Wire.beginTransmission(0x68);
  Wire.write(67);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  for(i = 0; i < 3; i++)
    gyro_raw[i] = gyro_raw[i] * 0.8 + 0.2 * (((Wire.read() << 8) | Wire.read()) - offset[i]);
  // Get DT
  static unsigned long p = 0;
  unsigned long c = micros();
  float dt = (c - p) * 0.000001F;
  p = c;
  // Gyro Rate
  float gyro_rate[3];
  for(i = 0; i < 3; i++) gyro_rate[i] = gyro_raw[i] / 16.4 * dt;
  // Calculate
  static float angle[3]={0,}, vec;
  vec = sqrt(pow(acc_raw[0], 2) + pow(acc_raw[2], 2));
  angle[0] = (angle[0] + gyro_rate[0]) * 0.98
    + atan2(acc_raw[1], vec) * RAD_TO_DEG * 0.02;
  vec = sqrt(pow(acc_raw[1], 2) + pow(acc_raw[2], 2));
  angle[1] = (angle[1] - gyro_rate[1]) * 0.98
    + atan2(acc_raw[0], vec) * RAD_TO_DEG * 0.02;
  // Serial print
  angle[2] += gyro_rate[2];
  char str[50], a1[10], a2[10], a3[10];
  dtostrf(angle[0], 4, 3, a1);
  dtostrf(angle[1], 4, 3, a2);
  dtostrf(angle[2], 4, 3, a3);
  sprintf(str, "X:%s Y:%s Z:%s", a1, a2, a3);
    String a1str = String(a1); 
    String a2str = String(a2); 
    int angx=a1str.toInt();
    int angy=a2str.toInt();
    ServoOutputx=anglex-1+angx;
    ServoOutputy=angley+5-angy;
    if(ServoOutputx>anglexlow&&ServoOutputx<anglexhigh){
      ServoOutputx=anglex;
    }
    if(ServoOutputy>angleylow&&ServoOutputy<angleyhigh){
      ServoOutputy=angley;
    }
    myservox.write(ServoOutputx);
    myservoy.write(ServoOutputy);   

    Serial.print(ServoOutputx);
    Serial.print(",");
    Serial.println(ServoOutputy);
}
