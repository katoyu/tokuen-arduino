#include <SPI.h>
#include <MsTimer2.h>
#include <MPU9250_asukiaaa.h>

// ピン定義:モータードライバー
#define PIN_SPI_MOSI (11)
#define PIN_SPI_MISO (12)
#define PIN_SPI_SCK (13)
#define PIN_SPI_SS (10)
#define PIN_BUSY 8

// ピン定義:センサ
#ifdef _ESP32_HAL_I2C_H_
#define SDA_PIN 26
#define SCL_PIN 25
#endif

#define D 5

//初期値セット
double Yamemo = 0.00;
double Yaheni = 0.00;
double Yamove = 0.00;
double Ygmemo = 0.00;
double Ygheni = 0.00;
double Ygmove = 0.00;
double Yset,Ynow,Ypast;
int count=0;
int swi=0; //0,1で変化　0の時正常動作　1の時異符号をシャットアウトしgyが0付近になったら解除

MPU9250 mySensor;

uint8_t sensorId;
float aX, aY, aZ, aSqrt, gX, gY, gZ, mDirection, mX, mY, mZ;

void setup()
{
delay(200);
pinMode(PIN_SPI_MOSI, OUTPUT);
pinMode(PIN_SPI_MISO, INPUT);
pinMode(PIN_SPI_SCK, OUTPUT);
pinMode(PIN_SPI_SS, OUTPUT);
pinMode(PIN_BUSY, INPUT);
SPI.begin();
SPI.setDataMode(SPI_MODE3);
SPI.setBitOrder(MSBFIRST);
Serial.begin(9600);
digitalWrite(PIN_SPI_SS, HIGH);

L6470_resetdevice(); //L6470リセット
L6470_setup(); //L6470を設定

MsTimer2::set(50, fulash);//シリアルモニター用のタイマー割り込み
MsTimer2::start();

//L6470_goto(0x2789);
delay(1000);

/*
L6470_move(1,1600);//指定方向に指定数ステップする 
L6470_move(0,800);//指定方向に指定数ステップする 
delay(2000);
*/
/*
L6470_busydelay(5000); //busyフラグがHIGHになってから、指定ミリ秒待つ。
L6470_run(0,10000);//指定方向に連続回転
delay(6000);
L6470_softstop();//回転停止、保持トルクあり
L6470_busydelay(5000);
L6470_goto(0x6789);//指定座標に最短でいける回転方向で移動
L6470_busydelay(5000);
L6470_run(0,0x4567);
delay(6000);
L6470_hardhiz();//回転急停止、保持トルクなし
*/
//ここからセンサをシリアルモニタ に移す部分
  while(!Serial);
  Serial.begin(115200);
  Serial.println("started");

#ifdef _ESP32_HAL_I2C_H_ // For ESP32
  Wire.begin(SDA_PIN, SCL_PIN); // SDA, SCL
#else
  Wire.begin();
#endif

  mySensor.setWire(&Wire);
  mySensor.beginAccel();
  mySensor.beginGyro();
  mySensor.beginMag();

  // You can set your own offset for mag values
  // mySensor.magXOffset = -50;
  // mySensor.magYOffset = -55;
  // mySensor.magZOffset = -10;

  sensorId = mySensor.readId();

  mySensor.accelUpdate();
  aX = mySensor.accelX();
  aY = mySensor.accelY();
  aZ = mySensor.accelZ();

  Yamove = aY*180;
    Serial.println(" accelY: " + String(aY) );

      if(Yamove >= 0){
        L6470_move(1,int(Yamove));
      }else if(Yamove < 0){
        L6470_move(0,int(-Yamove));
      }
  Yamemo = aY;
  /*
  L6470_goto(0x6000);//指定座標に最短でいける回転方向で移動
  L6470_busydelay(1000);
  L6470_gohome();//指定座標に最短でいける回転方向で移動
  L6470_busydelay(1000);
  */
}
void loop(){

  //Serial.println("sensorId: " + String(sensorId));

  mySensor.accelUpdate();
  aX = mySensor.accelX();
  aY = mySensor.accelY();
  aZ = mySensor.accelZ();
  aSqrt = mySensor.accelSqrt();
  //Serial.print(" accelX: " + String(aX) );
  Serial.print(" accelY: " + String(aY) );
  //Serial.print(" accelZ: " + String(aZ) );
  //Serial.print(" accelSqrt: " + String(aSqrt) );

  mySensor.gyroUpdate();
  gX = mySensor.gyroX();
  gY = mySensor.gyroY();
  gZ = mySensor.gyroZ();
  //Serial.print(" gyroX: " + String(gX) );
  Serial.print(" gyroY: " + String(gY) );
  //Serial.print(" gyroZ: " + String(gZ) );

  mySensor.magUpdate();
  mX = mySensor.magX();
  mY = mySensor.magY();
  mZ = mySensor.magZ();
  mDirection = mySensor.magHorizDirection();
  //Serial.print(" magX: " + String(mX) );
  //Serial.print(" magY: " + String(mY) );
  Serial.print(" magZ: " + String(mZ) );
  //Serial.print(" horizontal direction: " + String(mDirection) );

  Serial.print(" at " + String(millis()) + "ms" );
  //Serial.print(" "); // Add an empty line

  Serial.print(" Yheni: " + String(Yaheni));
  Serial.print(" Ymove: " + String(Yamove));
  Serial.print(" Yheni: " + String(Ygheni));
  Serial.print(" Ymove: " + String(Ygmove));
  
 // L6470_move(1,25600);//指定方向に指定数ステップする 
  //L6470_(0x6789);
  //L6470_move(0,);//指定方向に指定数ステップする 
  //delay(1000);
   double Yaval,sum = 0;
  for(int i=0;i<100;i++){
      aY = mySensor.accelY();
    sum += aY;
  }
  Yaval = sum/100;
  Yaheni = Yaval-Yamemo;
  Yamove = Yaheni*180*25600/360;

  if(Yaheni >= 0.03){
    L6470_move(1,int(Yamove));
  }else if(Yaheni < -0.03){
    L6470_move(0,int(-Yamove));
  }

  Serial.println("");
  Yamemo = aY;

//  Yheni = (gY - Ymemo); 
//  Ymemo = gY;
//  Ypast = Ynow;
  delay(D);
}

void L6470_setup(){
L6470_setparam_acc(0x40); //[R, WS] 加速度default 0x08A (12bit) (14.55*val+14.55[step/s^2])
L6470_setparam_dec(0x40); //[R, WS] 減速度default 0x08A (12bit) (14.55*val+14.55[step/s^2])
L6470_setparam_maxspeed(0x40); //[R, WR]最大速度default 0x041 (10bit) (15.25*val+15.25[step/s])
L6470_setparam_minspeed(0x01); //[R, WS]最小速度default 0x000 (1+12bit) (0.238*val[step/s])
L6470_setparam_fsspd(0x3ff); //[R, WR]μステップからフルステップへの切替点速度default 0x027 (10bit) (15.25*val+7.63[step/s])
L6470_setparam_kvalhold(0x20); //[R, WR]停止時励磁電圧default 0x29 (8bit) (Vs[V]*val/256)
L6470_setparam_kvalrun(0x20); //[R, WR]定速回転時励磁電圧default 0x29 (8bit) (Vs[V]*val/256)
L6470_setparam_kvalacc(0x20); //[R, WR]加速時励磁電圧default 0x29 (8bit) (Vs[V]*val/256)
L6470_setparam_kvaldec(0x20); //[R, WR]減速時励磁電圧default 0x29 (8bit) (Vs[V]*val/256)

L6470_setparam_stepmood(0x07); //ステップモードdefault 0x07 (1+3+1+3bit)
}

void fulash(){
  /*
Serial.print("0x");
Serial.print( L6470_getparam_abspos(),HEX);
Serial.print(" ");
Serial.print("0x");
Serial.println( L6470_getparam_speed(),HEX);
*/
}


//-----------------------RegisterCommand-------------------------

void L6470_setparam_abspos(long val){L6470_transfer(0x01,3,val);}
void L6470_setparam_elpos(long val){L6470_transfer(0x02,2,val);}
void L6470_setparam_mark(long val){L6470_transfer(0x03,3,val);}
void L6470_setparam_acc(long val){L6470_transfer(0x05,2,val);}
void L6470_setparam_dec(long val){L6470_transfer(0x06,2,val);}
void L6470_setparam_maxspeed(long val){L6470_transfer(0x07,2,val);}
void L6470_setparam_minspeed(long val){L6470_transfer(0x08,2,val);}
void L6470_setparam_fsspd(long val){L6470_transfer(0x15,2,val);}
void L6470_setparam_kvalhold(long val){L6470_transfer(0x09,1,val);}
void L6470_setparam_kvalrun(long val){L6470_transfer(0x0a,1,val);}
void L6470_setparam_kvalacc(long val){L6470_transfer(0x0b,1,val);}
void L6470_setparam_kvaldec(long val){L6470_transfer(0x0c,1,val);}
void L6470_setparam_intspd(long val){L6470_transfer(0x0d,2,val);}
void L6470_setparam_stslp(long val){L6470_transfer(0x0e,1,val);}
void L6470_setparam_fnslpacc(long val){L6470_transfer(0x0f,1,val);}
void L6470_setparam_fnslpdec(long val){L6470_transfer(0x10,1,val);}
void L6470_setparam_ktherm(long val){L6470_transfer(0x11,1,val);}
void L6470_setparam_ocdth(long val){L6470_transfer(0x13,1,val);}
void L6470_setparam_stallth(long val){L6470_transfer(0x14,1,val);}
void L6470_setparam_stepmood(long val){L6470_transfer(0x16,1,val);}
void L6470_setparam_alareen(long val){L6470_transfer(0x17,1,val);}
void L6470_setparam_config(long val){L6470_transfer(0x18,2,val);}

long L6470_getparam_abspos(){return L6470_getparam(0x01,3);}
long L6470_getparam_elpos(){return L6470_getparam(0x02,2);}
long L6470_getparam_mark(){return L6470_getparam(0x03,3);}
long L6470_getparam_speed(){return L6470_getparam(0x04,3);}
long L6470_getparam_acc(){return L6470_getparam(0x05,2);}
long L6470_getparam_dec(){return L6470_getparam(0x06,2);}
long L6470_getparam_maxspeed(){return L6470_getparam(0x07,2);}
long L6470_getparam_minspeed(){return L6470_getparam(0x08,2);}
long L6470_getparam_fsspd(){return L6470_getparam(0x15,2);}
long L6470_getparam_kvalhold(){return L6470_getparam(0x09,1);}
long L6470_getparam_kvalrun(){return L6470_getparam(0x0a,1);}
long L6470_getparam_kvalacc(){return L6470_getparam(0x0b,1);}
long L6470_getparam_kvaldec(){return L6470_getparam(0x0c,1);}
long L6470_getparam_intspd(){return L6470_getparam(0x0d,2);}
long L6470_getparam_stslp(){return L6470_getparam(0x0e,1);}
long L6470_getparam_fnslpacc(){return L6470_getparam(0x0f,1);}
long L6470_getparam_fnslpdec(){return L6470_getparam(0x10,1);}
long L6470_getparam_ktherm(){return L6470_getparam(0x11,1);}
long L6470_getparam_adcout(){return L6470_getparam(0x12,1);}
long L6470_getparam_ocdth(){return L6470_getparam(0x13,1);}
long L6470_getparam_stallth(){return L6470_getparam(0x14,1);}
long L6470_getparam_stepmood(){return L6470_getparam(0x16,1);}
long L6470_getparam_alareen(){return L6470_getparam(0x17,1);}
long L6470_getparam_config(){return L6470_getparam(0x18,2);}
long L6470_getparam_status(){return L6470_getparam(0x19,2);}

//-----------------------CommandSet----------------------------------

void L6470_run(int dia,long spd){
if(dia==1)
L6470_transfer(0x51,3,spd);
else
L6470_transfer(0x50,3,spd);
}
void L6470_stepclock(int dia){
if(dia==1)
L6470_transfer(0x59,0,0); 
else
L6470_transfer(0x58,0,0);
}
void L6470_move(int dia,long n_step){
if(dia==1)
L6470_transfer(0x41,3,n_step);
else
L6470_transfer(0x40,3,n_step);
}
void L6470_goto(long pos){
L6470_transfer(0x60,3,pos);
}
void L6470_dia(int dia,int pos){
if(dia==1) 
L6470_transfer(0x69,3,pos);
else 
L6470_transfer(0x68,3,pos);
}
void L6470_gountil(int act,int dia,long spd){
if(act==1)
if(dia==1)
L6470_transfer(0x8b,3,spd);
else
L6470_transfer(0x8a,3,spd);
else
if(dia==1)
L6470_transfer(0x83,3,spd);
else
L6470_transfer(0x82,3,spd);
} 
void L6470_relesesw(int act,int dia){
if(act==1)
if(dia==1)
L6470_transfer(0x9b,0,0);
else
L6470_transfer(0x9a,0,0);
else
if(dia==1)
L6470_transfer(0x93,0,0);
else
L6470_transfer(0x92,0,0);
}
void L6470_gohome(){
L6470_transfer(0x70,0,0);
}
void L6470_gomark(){
L6470_transfer(0x78,0,0);
}
void L6470_resetpos(){
L6470_transfer(0xd8,0,0);
}
void L6470_resetdevice(){
L6470_send_u(0x00);//nop命令
L6470_send_u(0x00);
L6470_send_u(0x00);
L6470_send_u(0x00);
L6470_send_u(0xc0);
}
void L6470_softstop(){
L6470_transfer(0xb0,0,0);
}
void L6470_hardstop(){
L6470_transfer(0xb8,0,0);
}
void L6470_softhiz(){
L6470_transfer(0xa0,0,0);
}
void L6470_hardhiz(){
L6470_transfer(0xa8,0,0);
}
long L6470_getstatus(){
long val=0;
L6470_send_u(0xd0);
for(int i=0;i<=1;i++){
val = val << 8;
digitalWrite(PIN_SPI_SS, LOW); // ~SSイネーブル。
val = val | SPI.transfer(0x00); // アドレスもしくはデータ送信。
digitalWrite(PIN_SPI_SS, HIGH); // ~SSディスエーブル 
}
return val;
}

void L6470_transfer(int add,int bytes,long val){
int data[3];
L6470_send(add);
for(int i=0;i<=bytes-1;i++){
data[i] = val & 0xff; 
val = val >> 8;
}
if(bytes==3){
L6470_send(data[2]);
}
if(bytes>=2){
L6470_send(data[1]);
}
if(bytes>=1){
L6470_send(data[0]);
} 
}
void L6470_send(unsigned char add_or_val){
while(!digitalRead(PIN_BUSY)){
} //BESYが解除されるまで待機
digitalWrite(PIN_SPI_SS, LOW); // ~SSイネーブル。
SPI.transfer(add_or_val); // アドレスもしくはデータ送信。
digitalWrite(PIN_SPI_SS, HIGH); // ~SSディスエーブル。
}
void L6470_send_u(unsigned char add_or_val){//busyを確認せず送信するため用
digitalWrite(PIN_SPI_SS, LOW); // ~SSイネーブル。
SPI.transfer(add_or_val); // アドレスもしくはデータ送信。
digitalWrite(PIN_SPI_SS, HIGH); // ~SSディスエーブル。
}
void L6470_busydelay(long time){//BESYが解除されるまで待機
while(!digitalRead(PIN_BUSY)){
}
delay(time);
}
long L6470_getparam(int add,int bytes){
long val=0;
int send_add = add | 0x20;
L6470_send_u(send_add);
for(int i=0;i<=bytes-1;i++){
val = val << 8;
digitalWrite(PIN_SPI_SS, LOW); // ~SSイネーブル。
val = val | SPI.transfer(0x00); // アドレスもしくはデータ送信。
digitalWrite(PIN_SPI_SS, HIGH); // ~SSディスエーブル 
}
return val;
}
