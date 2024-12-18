// ピン設定
const int AIN1 = 4;
const int AIN2 = 2;
const int BIN1 = 27;
const int BIN2 = 26;
const int STBY = 25;  // LOWにするとスリープモードになる
int previousDirection = 0;

float integral = 0;  // 積分初期値
float lastError = 0; // 1ステップ前の初期値
float Kp = 10;      // 比例定数
float Ki = 0.4;        // 積分定数
float Kd = 2.5;        // 微分定数
float baseSpeed = 100; //モーター回転速度初期値

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

}

//PID制御関数
float PIDControl(int error){
  // P成分
  float proportional = Kp * error;

  // I成分
  integral += error;
  float integralCorrection = Ki * integral;

  // D成分
  float derivative = Kd * (error - lastError);
  lastError = error;

  // 最終補正量
  correction = (proportional + integralCorrection + derivative) * 0.5;
  
  return correction;
}

//障害物検知関数
//2つのセンサで障害物を検知して
int detectObstacle(){
  int Distance;
  return Distance;
}


unsigned long previousTime = 0;
const unsigned long interval = 10;
float correction;

void loop() {
  // フォトリフレクタの値読み取り
  // 光が反射しない、黒 → 値0
  // 光が反射する、白 → 値4095
  int s1 = analogRead(14);
  int s2 = analogRead(33);
  int s3 = analogRead(32);
  int s4 = analogRead(34);
  int s5 = analogRead(35);

  int threshold = 2000;
  int position = (s1 < threshold ? -3 : 0) + 
                 (s2 < threshold ? -1 : 0) + 
                 (s3 < threshold ?  0 : 0) + 
                 (s4 < threshold ?  1 : 0) + 
                 (s5 < threshold ?  3 : 0);

  int light_count = (s1 < threshold ? 1 : 0) + 
                 (s2 < threshold ? 1 : 0) + 
                 (s3 < threshold ?  1 : 0) + 
                 (s4 < threshold ?  1 : 0) + 
                 (s5 < threshold ?  1 : 0);

  unsigned long currentTime = millis();
  if (currentTime - previousTime >= interval) {
    previousTime = currentTime;

    float error = position / light_count;  

    float correction = PIDContorol(error);


  motor.motorDRV8833_R(baseSpeed - correction);
  motor.motorDRV8833_L(baseSpeed + correction);

  if (s1 > threshold && s2 > threshold && s3 > threshold && s4 > threshold && s5 > threshold) {
    Serial.println("ライン外れ");
    motor.motorDRV8833_R(-100);
    motor.motorDRV8833_L(100);
  }
}