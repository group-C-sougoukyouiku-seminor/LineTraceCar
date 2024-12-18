// ピン設定
const int AIN1 = 4;
const int AIN2 = 2;
const int BIN1 = 27;
const int BIN2 = 26;
const int STBY = 25;  // LOWにするとスリープモードになる
int previousDirection = 0;

unsigned long currentTime, previousTime;
float deltaTime;

float integral = 0;  // グローバル変数
float lastError = 0; // グローバル変数
float Kp = 0.2;
float Ki = 0.05;
float Kd = 0.1;

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
  previousTime = millis();

}

void loop() {
    currentTime = millis();
    deltaTime = (currentTime - previousTime) / 1000.0;
    previousTime = currentTime;

  // フォトリフレクタの値読み取り
  // 光が反射しない、黒 → 値0
  // 光が反射する、白 → 値4095
  int s1 = analogRead(14);
  int s2 = analogRead(33);
  int s3 = analogRead(32);
  int s4 = analogRead(34);
  int s5 = analogRead(35);

  String checkThreshold =  " [ " +  String(s1) + " / " + String(s2) + " / " + String(s3) + " / " + String(s4) + " / " + String(s5) + " ] "; 
  Serial.println(checkThreshold);

  int threshold = 2000;
  int position = (s1 < threshold ? -2 : 0) + 
                 (s2 < threshold ? -1 : 0) + 
                 (s3 < threshold ?  0 : 0) + 
                 (s4 < threshold ?  1 : 0) + 
                 (s5 < threshold ?  2 : 0);

  float error = position;  

    // P成分
  float proportional = Kp * error;

  // I成分
  integral += error;
  float integralCorrection = Ki * integral;

  // D成分
  float derivative = Kd * ((error - lastError) / deltaTime);
  lastError = error;

  // 最終補正量
  float correction = (proportional + integralCorrection + derivative) * 1;

  Serial.print("correction: ");
  Serial.println(correction);

  float baseSpeed = 200;
  
  motor.motorDRV8833_R(baseSpeed - correction);
  motor.motorDRV8833_L(baseSpeed + correction);

  if (s1 > threshold && s2 > threshold && s3 > threshold && s4 > threshold && s5 > threshold) {
    Serial.println("ライン外れ");
    motor.motorDRV8833_R(-50); // ゆっくり後退
    motor.motorDRV8833_L(-50);
    // delay(500);
    // motor.motorDRV8833_R(0); // 停止
    // motor.motorDRV8833_L(0);
  }

}