#include <Wire.h>
#include <MsTimer2.h>
#include <TimerOne.h>

#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x3F,16,2);


volatile float k1 = 300, k2 = 25, k3 = 10, k4 = 1;

#define A_DIR   4
#define A_STEP  5
#define A_EN    6
#define B_DIR   7
#define B_STEP  8
#define B_EN    9
#define C_DIR   10
#define C_STEP  11
#define C_EN    12

#define PULSE_PERIOD  20 // パルス生成のためのTimer1のタイマ割り込み周期 マイクロ秒単位
#define CONTROL_PERIOD 4 // 制御ループの周期 ミリ秒単位
#define LIMIT 560        // ステッピングモータが脱調しない最大のスピード
#define MICRO_STEP 4     // ステッピングモータのマイクロステップの逆数 (1/4 -> 4)

//倒立振子の制御で使用する変数
volatile long lastUptime, startTime, currentTime;
volatile float dt;

//Y軸方向の制御で使用する変数
volatile int16_t rawGyroY;
volatile float caribGyroY;
volatile float gyroY;
volatile float degY = 0;
volatile float dpsY = 0;
volatile int speedY = 0;
volatile float rotY = 0;
volatile float lpfY = 0;
volatile float lpfYA = 0.9999;

//X軸方向の制御で使用する変数
volatile int16_t rawGyroX;
volatile float caribGyroX;
volatile float gyroX;
volatile float degX = 0;
volatile float dpsX = 0;
volatile int speedX = 0;
volatile float rotX = 0;
volatile float lpfX = 0;
volatile float lpfXA = 0.9999;

//ステッピングモータの制御で使用する変数
volatile int countA = 0;
volatile int speedA = 0;
volatile int countB = 0;
volatile int speedB = 0;
volatile int countC = 0;
volatile int speedC = 0;

// 加速度・ジャイロセンサーの制御定数
#define MPU6050_ADDR         0x68 // MPU-6050 device address
#define MPU6050_SMPLRT_DIV   0x19 // MPU-6050 register address
#define MPU6050_CONFIG       0x1a
#define MPU6050_GYRO_CONFIG  0x1b
#define MPU6050_ACCEL_CONFIG 0x1c
#define MPU6050_ACCEL_XOUT_H 0x3b
#define MPU6050_ACCEL_XOUT_L 0x3c
#define MPU6050_ACCEL_YOUT_H 0x3d
#define MPU6050_ACCEL_YOUT_L 0x3e
#define MPU6050_ACCEL_ZOUT_H 0x3f
#define MPU6050_ACCEL_ZOUT_L 0x40
#define MPU6050_GYRO_XOUT_H  0x43
#define MPU6050_GYRO_XOUT_L  0x44
#define MPU6050_GYRO_YOUT_H  0x45
#define MPU6050_GYRO_YOUT_L  0x46
#define MPU6050_GYRO_ZOUT_H  0x47
#define MPU6050_GYRO_ZOUT_L  0x48
#define MPU6050_PWR_MGMT_1   0x6b
#define MPU6050_WHO_AM_I     0x75

// センサーへのコマンド送信
void writeMPU6050(byte reg, byte data) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

// センサーからのデータ読み込み
byte readMPU6050(byte reg) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 1, false);
  //readで読み取れるバイト数がなければLED13を消灯
  while (! Wire.available()) {
    digitalWrite(13, LOW);
  }
  byte data =  Wire.read();
  Wire.endTransmission(true);
  return data;
}

void setup() {
  Wire.setClock(400000);
  Serial.begin(115200);
  Serial.println("*******************RESTARTED********************");
  Serial.println("*********************ballbot***********************");

  Serial.println(k1);
  Serial.println(k2);
  Serial.println(k3);
  Serial.println(k4);

  lcd.init(); 
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Ballbot!!");

  // モータードライバ制御用ピンの初期化
  pinMode(A_DIR, OUTPUT);
  pinMode(A_STEP, OUTPUT);
  pinMode(A_EN, OUTPUT);
  pinMode(B_DIR, OUTPUT);
  pinMode(B_STEP, OUTPUT);
  pinMode(B_EN, OUTPUT);
  pinMode(C_DIR, OUTPUT);
  pinMode(C_STEP, OUTPUT);
  pinMode(C_EN, OUTPUT);

  //状態をLED 13にて表示
  pinMode(13, OUTPUT);

  // センサーの初期化
  Wire.begin();
  if (readMPU6050(MPU6050_WHO_AM_I) != 0x68) {
    Serial.println("WHO_AM_I error.");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("WHO_AM_I error.");
    while (true) ;
  }
  else {
    Serial.println("WHO_AM_I OK.");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("WHO_AM_I OK.");
  }
  // see https://www.invensense.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
  writeMPU6050(MPU6050_SMPLRT_DIV, 0x07);   // sample rate: 8kHz/(7+1) = 1kHz
  writeMPU6050(MPU6050_CONFIG, 0x00);       // disable DLPF, gyro output rate = 8kHz
  writeMPU6050(MPU6050_GYRO_CONFIG, 0x00);  // gyro range: 0x00⇒±250dps 131LSB、0x08⇒±500dps 65.5LSB、0x10⇒±1000dps 32.8LSB、0x18⇒±2000dps 16.4LSB
  writeMPU6050(MPU6050_ACCEL_CONFIG, 0x00); // accel range: 0x00⇒±2g 16384LSB/g、0x01⇒±4g 8192LSB/g、0x02⇒±8g 4096LSB/g、0x03⇒±16g 2048LSB/g、
  writeMPU6050(MPU6050_PWR_MGMT_1, 0x01);   // disable sleep mode, PLL with X gyro
  Serial.println("MPU6050 Setup OK.");
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("MPU6050 Setup OK.");
  delay(2000);

  //ジャイロのゼロ点調整のために静止時の出力を1000回計測して平均を算出
  caribGyroX = 0;
  caribGyroY = 0;
  for (int i = 0; i < 1000  ; i++)  {
    rawGyroX = (readMPU6050(MPU6050_GYRO_XOUT_H) << 8) | readMPU6050(MPU6050_GYRO_XOUT_L);
    rawGyroY = (readMPU6050(MPU6050_GYRO_YOUT_H) << 8) | readMPU6050(MPU6050_GYRO_YOUT_L);
    caribGyroX += (float) rawGyroX;
    caribGyroY += (float) rawGyroY;
  }
  caribGyroX /= 1000;
  caribGyroY /= 1000;
  Serial.println("Carib OK.");
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Carib OK.");

  //倒立時間、dt計測用
  currentTime = micros();
  startTime = currentTime;
  lastUptime = currentTime;

  digitalWrite(A_EN, LOW);
  digitalWrite(B_EN, LOW);
  digitalWrite(C_EN, LOW);

  // タイマ割込みを始める前にLDC表示をする
  digitalWrite(13, HIGH);
  Serial.println("******************** GO !! *********************");
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("GO!!");

  Timer1.initialize(PULSE_PERIOD); //パルス生成のためのタイマ割込み 引数はマイクロ秒単位
  Timer1.attachInterrupt(pulse);
  MsTimer2::set(CONTROL_PERIOD, controlloop); //制御のためのタイマ割込み 引数はミリ秒単位
  MsTimer2::start();
}

void pulse() {
  countA += speedA*MICRO_STEP;
  if (countA > 5000) {
    PORTD ^= B00100000;
    countA -= 5000;
  }
  else if (countA < -5000) {
    PORTD ^= B00100000;
    countA += 5000;
  }
  countB += speedB*MICRO_STEP;
  if (countB > 5000) {
    PORTB ^= B00000001;
    countB -= 5000;
  }
  else if (countB < -5000) {
    PORTB ^= B00000001;
    countB += 5000;
  }
  countC += speedC*MICRO_STEP;
  if (countC > 5000) {
    PORTB ^= B00001000;
    countC -= 5000;
  }
  else if (countC < -5000) {
    PORTB ^= B00001000;
    countC += 5000;
  }
}

void controlloop () {
  // 割り込みハンドラの中でI2C通信(割り込み処理を使用）を許可する
  interrupts();

  // dt計測
  currentTime = micros();
  dt = (currentTime - lastUptime) * 0.000001;
  lastUptime = currentTime;

  // 角速度を取得
  rawGyroX =  (readMPU6050(MPU6050_GYRO_XOUT_H) << 8) | readMPU6050(MPU6050_GYRO_XOUT_L);
  gyroX = (float) rawGyroX - caribGyroX;
  dpsX = gyroX / 131.0;
  
  rawGyroY =  (readMPU6050(MPU6050_GYRO_YOUT_H) << 8) | readMPU6050(MPU6050_GYRO_YOUT_L);
  gyroY = (float) rawGyroY - caribGyroY;
  dpsY = gyroY / 131.0;

  // 角速度を積算して角度を求める
  degX +=  dpsX * dt;
  degY +=  dpsY * dt;

  Serial.print(degX);
  Serial.print(",");
  Serial.println(degY);
  
  // 速度を積算して回転角を求める
  rotX += speedX * dt;
  rotY += speedY * dt;

  //ローパスフィルタ
  lpfX *=  lpfXA;
  lpfX +=  (1 - lpfXA) * degX;
  lpfY *=  lpfYA;
  lpfY +=  (1 - lpfYA) * degY;

  // 制御量の計算
  speedX += (k1 * (degX - lpfX) + k2 * dpsX + k3 * rotX + k4 * speedX) * dt;
  speedY += (k1 * (degY - lpfY) + k2 * dpsY + k3 * rotY + k4 * speedY) * dt;

  // ステッピングモータの制御に変換
  speedC = 0                -       speedX;
  speedA = - 0.866 * speedY + 0.5 * speedX;
  speedB = + 0.866 * speedY + 0.5 * speedX;

  // ステッピングモータの最大速度を制限
  speedA = constrain(speedA, 0 - LIMIT, LIMIT);
  speedB = constrain(speedB, 0 - LIMIT, LIMIT);
  speedC = constrain(speedC, 0 - LIMIT, LIMIT);

  digitalWrite(A_DIR, (speedA < 0));
  digitalWrite(B_DIR, (speedB < 0));
  digitalWrite(C_DIR, (speedC < 0));

  // 倒れたらモーター停止
  if (20 < abs(degX - lpfX) || 20 < abs(degY - lpfY)) {
    speedX = 0;
    speedY = 0;
    digitalWrite(A_EN, HIGH);
    digitalWrite(B_EN, HIGH);
    digitalWrite(C_EN, HIGH);
    Serial.println("********************stopped*********************");
    MsTimer2::stop();

    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Stopped.");
    lcd.setCursor(0,1);
    lcd.print((currentTime - startTime) / 1000000);

    while (1) {
      digitalWrite(13, LOW);
      delay(500);
      digitalWrite(13, HIGH);
      delay(500);
    }
  }
}

void loop() {
}
