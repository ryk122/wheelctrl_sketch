//ホイールコントローラ計画
//v1.1(公開後2回目) 2019/09/08
//programed by ryk
//このコードの使用は自己責任でお願いします。

#include <Joystick.h>
#include <Wire.h>

#define uppin 4   //shift up
#define downpin 6 //shift down
#define echoPinA 16 // Echo Pin A
#define trigPinA 10 // Trigger Pin A
#define echoPinB 8 // Echo Pin B
#define trigPinB 9 // Trigger Pin B
#define g_max 0.9

Joystick_ Joystick;

float aopoint; //accele 0 point
float ampoint; //accele full point
float bopoint; //brake 0 point
float bmpoint; //brake full point

float lsteer;

void setup() {
  // put your setup code here, to run once:
  pinMode(uppin, INPUT);
  pinMode(downpin, INPUT);
  pinMode( echoPinA, INPUT );
  pinMode( trigPinA, OUTPUT );
  pinMode( echoPinB, INPUT );
  pinMode( trigPinB, OUTPUT );
  Serial.begin(9600);

  //set gsensor
  Wire.begin();
  setupMPU6050();

  //joystick set
  Joystick.begin();
  Joystick.setXAxis(511);
  Joystick.setYAxis(511);
  Joystick.setZAxis(511);
  
  //set 0 point of range
  while(1){
    if(digitalRead(uppin)==HIGH){
      aopoint = sonicmeasure(trigPinA,echoPinA);
      bopoint = sonicmeasure(trigPinB,echoPinB);
      break;
    }
  }
  while(1){
    if(digitalRead(uppin)==LOW)
      break;
  }
  //set Full point of range
  while(1){
    if(digitalRead(uppin)==HIGH){
      ampoint = sonicmeasure(trigPinA,echoPinA);
      bmpoint = sonicmeasure(trigPinB,echoPinB);
      break;
    }
  }
}

void loop() {
  //steer
  float steer = fmap(recordAccelRegisters(),-g_max,g_max);
  /*
  if(abs(lsteer-steer)>0.3)
    steer=lsteer;
  else
    lsteer=steer;
    */ 
  if(steer>1) steer=1; else if(steer<0) steer=0;
  Joystick.setXAxis(1023*steer);
  Serial.println(1023*steer);
  
  // accele and brake
  float ac = fmap(sonicmeasure(trigPinA,echoPinA),aopoint,ampoint);
  if(ac < 0) ac = 0; else if (ac > 0.93) ac = 1;
  float br = fmap(sonicmeasure(trigPinB,echoPinB),bopoint,bmpoint);
  if(br < 0) br = 0; else if (br > 0.93) br = 1;
  Joystick.setYAxis(511+ac*512);
  Joystick.setZAxis(511+br*512);
  /*
  Serial.print(ac);
  Serial.print("   ");
  Serial.println(br);*/

  //button
  if(digitalRead(uppin)==HIGH){
    Joystick.setButton(0,1);
  }
  else{
    Joystick.setButton(0,0);
  }
  if(digitalRead(downpin)==HIGH){
    Joystick.setButton(1,1);
  }
  else{
    Joystick.setButton(1,0);
  }
}

float fmap(float x, float a, float b){
  //change range (a ~ b) to (0 ~ 1)
  return ((x-a)/(b-a));
}

float sonicmeasure(int t, int e){
  digitalWrite(t, LOW); 
  delayMicroseconds(2); 
  digitalWrite( t, HIGH ); //超音波を出力
  delayMicroseconds( 10 ); //
  digitalWrite( t, LOW );
  return pulseIn( e, HIGH );
}

void setupMPU6050() {
  //MPU6050との通信を開始し、ジャイロと加速度の最大範囲を指定
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x6B); //Accessing the register 6B
  Wire.write(0b00000000); //SLEEP register to 0
  Wire.endTransmission();
  Wire.beginTransmission(0b1101000);
  Wire.write(0x1B); //Accessing the register 1B - Gyroscope Configuration
  Wire.write(0x00000000); //gyro to full scale ± 250deg./s
  Wire.endTransmission();
  Wire.beginTransmission(0b1101000);
  Wire.write(0x1C); //Accessing the register 1C - Acccelerometer Configuration
  Wire.write(0b00000000); //accel to +/- 2g
  Wire.endTransmission();
}

float recordAccelRegisters() {
  //加速度読み取り
  Wire.beginTransmission(0b1101000);
  Wire.write(0x3B); // Accel Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000, 6); //Request Accel Registers (3B - 40)
  while (Wire.available() < 6);
  float accelX = Wire.read() << 8 | Wire.read(); //Store first two bytes into accelX
  float accelY = Wire.read() << 8 | Wire.read(); //Store middle two bytes into accelY
  float accelZ = Wire.read() << 8 | Wire.read(); //Store last two bytes into accelZ

  //省略して、yだけ返す
  return accelY / 16384.0;
}
