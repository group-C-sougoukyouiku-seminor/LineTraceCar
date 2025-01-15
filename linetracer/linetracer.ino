//整数型2個の構造体の定義
#define trigPin 18  // トリガーピン  超音波を送信するための信号を送信
#define echoPin 19  // エコーピン  反射音波を受信するしてものを読み取るためのピン

float Duration = 0;  // 計測した時間
float Distance = 0;  // 距離

struct DoubleResult{
  int position;
  int lightCount;
};


// ピン設定
const int AIN1 = 4;
const int AIN2 = 2;
const int BIN1 = 27;
const int BIN2 = 26;
const int STBY = 25;  // LOWにするとスリープモードになる
int previousDirection = 0;

int step = 0; //ステップ数
float integral = 0;  // 積分初期値
float lastError = 0; // 1ステップ前の初期値
float Kp = 100;      // 比例定数
float Ki = 6;        // 積分定数
float Kd = 3;        // 微分定数
float PIDvalue = 0.3; //PID制御の倍率
float baseSpeed = 150; //モーター回転速度初期値
int threshold = 2000; //フォトリフレクタ反応閾値
float maxSpeed = 255;
float minSpeed = 0;


class DRV8833 {
  private:
    const int freq = 10000;  // 周波数[Hz]
    const int resolution = 8;  // 分解能[bit]

  public:
    DRV8833(int AIN1, int AIN2, int BIN1, int BIN2, int STBY) {
      pinMode(STBY, OUTPUT);
      digitalWrite(STBY, HIGH);  // STBYを有効化

      // PWMセットアップ
      ledcAttach(AIN1, freq, resolution);
      ledcAttach(AIN2, freq, resolution);
      ledcAttach(BIN1, freq, resolution);
      ledcAttach(BIN2, freq, resolution);
    }

    void motorDRV8833_L(int speed) {
      if (speed > 0 && speed < 256) {
        ledcWrite(AIN1, 255 - speed);
        ledcWrite(AIN2, 255);
      } else if (speed == 0) {
        ledcWrite(AIN1, 255);
        ledcWrite(AIN2, 255);
      } else if (speed > -256 && speed < 0) {
        ledcWrite(AIN1, 255);
        ledcWrite(AIN2, 255 + speed);
      } else {
        ledcWrite(AIN1, 255);
        ledcWrite(AIN2, 255);
      }
    }

    void motorDRV8833_R(int speed) {
      if (speed > 0 && speed < 256) {
        ledcWrite(BIN2, 255 - speed);
        ledcWrite(BIN1, 255);
      } else if (speed == 0) {
        ledcWrite(BIN2, 255);
        ledcWrite(BIN1, 255);
      } else if (speed > -256 && speed < 0) {
        ledcWrite(BIN2, 255);
        ledcWrite(BIN1, 255 + speed);
      } else {
        ledcWrite(BIN2, 255);
        ledcWrite(BIN1, 255);
      }
    }
};

DRV8833 motor(AIN1, AIN2, BIN1, BIN2, STBY);

void setup() {
  Serial.begin(115200);
  // previousDirection = 0;

  pinMode(echoPin,INPUT);   // エコーピンを入力に
  pinMode(trigPin,OUTPUT);  // トリガーピンを出力に

  pinMode(15, OUTPUT);

}

//PID制御関数
float PIDControl(float error){
  // P成分
  float proportional = Kp * error;

  // I成分
  integral += error;
  float integralCorrection = Ki * integral;

  // D成分
  float derivative = Kd * (error - lastError);
  lastError = error;

  // 最終補正量
  float correction = (proportional + integralCorrection + derivative) * PIDvalue;
  
  return correction;
}

//障害物検知関数
//2つのセンサで障害物を検知して
// int detectObstacle(){
//   int Distance;
//   return Distance;
// }

DoubleResult photoReflector(int s1, int s2, int s3, int s4, int s5){
  DoubleResult PhotoReflector;
  // フォトリフレクタの値読み取り
  // 光が反射しない、黒 → 値0
  // 光が反射する、白 → 値4095
  PhotoReflector.position = (s1 < threshold ? -3 : 0) +
                            (s2 < threshold ? -1 : 0) +
                            (s3 < threshold ? 0 : 0) +
                            (s4 < threshold ? 1 : 0) +
                            (s5 < threshold ? 3 : 0);

  PhotoReflector.lightCount = (s1 < threshold ? 1 : 0) + 
                              (s2 < threshold ? 1 : 0) + 
                              (s3 < threshold ?  1 : 0) + 
                              (s4 < threshold ?  1 : 0) + 
                              (s5 < threshold ?  1 : 0);

  return PhotoReflector;
}

//時間計測
unsigned long previousTime = 0;
const unsigned long interval = 10; //PID制御を回す周期
float correction;


//メインの文章
void loop(){
  digitalWrite(trigPin,LOW);              // 計測前に一旦トリガーピンをLowに
  delayMicroseconds(2);
  digitalWrite(trigPin,HIGH);             // トリガーピンを10μsの時間だけHighに→センサーから超音波が出る
  delayMicroseconds(10);
  digitalWrite(trigPin,LOW);
  ////////////////////////
  // 超音波が送信されてから戻ってくるまでの時間[μs] 
  Duration = pulseIn(echoPin,HIGH);      //  pulseInは特定のパルス(今回はHIGH)の長さをμsで返す

  /// パルスの長さから物体までの距離[mm]を算出 ///
  Duration = Duration / 2;               // 距離を1/2に
  Distance = Duration*340*1000/1000000 / 10;  

  int s1 = analogRead(14);
  int s2 = analogRead(33);
  int s3 = analogRead(32);
  int s4 = analogRead(34);
  int s5 = analogRead(35);

  DoubleResult PhotoReflector = photoReflector(s1, s2, s3, s4, s5);

  unsigned long currentTime = millis();

if (Distance < 15 && Distance != 0.0) {
    Serial.println("障害物検知！");
    Serial.println(Distance);

    // 障害物検知 → 停止
    motor.motorDRV8833_R(0);
    motor.motorDRV8833_L(0);
    delay(500); 

    // ちょい右に曲がる
    Serial.println("Turning Right");
    motor.motorDRV8833_R(-150); // 右モーター後退
    motor.motorDRV8833_L(150);  // 左モーター前進
    delay(200);  

    // ちゃんと前進
    Serial.println("Moving Forward");
    motor.motorDRV8833_R(150);  // 右モーター前進
    motor.motorDRV8833_L(150);  // 左モーター前進
    delay(1000); 
    
}

  if (currentTime - previousTime >= interval) {
    previousTime = currentTime;

  float error = (PhotoReflector.lightCount != 0) ? 
               (PhotoReflector.position / PhotoReflector.lightCount) : 0; 

    float correction = PIDControl(error);

    float RmotorSpeed = baseSpeed - correction;
    float LmotorSpeed = baseSpeed + correction;


    if (RmotorSpeed >= maxSpeed){
      RmotorSpeed = maxSpeed;
    } 
    if (LmotorSpeed >= maxSpeed){
      LmotorSpeed = maxSpeed;
    } 
    if (RmotorSpeed <= minSpeed){
      RmotorSpeed = minSpeed;
    } 
    if (LmotorSpeed <= minSpeed){
      LmotorSpeed = minSpeed;
    } 

    Serial.println(RmotorSpeed);
    Serial.println(LmotorSpeed);

    motor.motorDRV8833_R(RmotorSpeed);
    motor.motorDRV8833_L(LmotorSpeed);
  }

  if (s1 > threshold && s2 > threshold && s3 > threshold && s4 > threshold && s5 > threshold) {
    Serial.println("ライン外れ");
    motor.motorDRV8833_R(0);
    motor.motorDRV8833_L(0);
  }
}