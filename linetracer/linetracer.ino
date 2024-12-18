//整数型2個の構造体の定義
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
float Kp = 9;      // 比例定数
float Ki = 0.4;        // 積分定数
float Kd = 1.5;        // 微分定数
float PIDvalue = 0.75; //PID制御の倍率
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

}

const int BUFFER_SIZE = 100; // バッファサイズ（十分な値を選択）
float errorBuffer[BUFFER_SIZE];  // 誤差を記録する配列
unsigned long timeBuffer[BUFFER_SIZE]; // タイムスタンプを記録する配列
int bufferIndex = 0; // 現在のバッファインデックス

float calculateIntegral(float currentError, unsigned long currentTime) {
  // 現在の誤差と時刻をバッファに記録
  errorBuffer[bufferIndex] = currentError;
  timeBuffer[bufferIndex] = currentTime;
  bufferIndex = (bufferIndex + 1) % BUFFER_SIZE; // リングバッファのインデックス管理

  // 過去3秒以内の誤差を合計
  float integralSum = 0;
  for (int i = 0; i < BUFFER_SIZE; i++) {
    if (currentTime - timeBuffer[i] <= 3000) { // 3秒以内のデータのみ
      integralSum += errorBuffer[i];
    }
  }

  return integralSum;
}

// PID制御関数（修正版）
float PIDControl(float error) {
  unsigned long currentTime = millis();

  // P成分
  float proportional = Kp * error;

  // I成分（過去3秒間の誤差のみを考慮）
  integral = calculateIntegral(error, currentTime);
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
int detectObstacle(){
  int Distance;
  return Distance;
}

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
  int s1 = analogRead(14);
  int s2 = analogRead(33);
  int s3 = analogRead(32);
  int s4 = analogRead(34);
  int s5 = analogRead(35);

  DoubleResult PhotoReflector = photoReflector(s1, s2, s3, s4, s5);

  unsigned long currentTime = millis();
  if (currentTime - previousTime >= interval) {
    previousTime = currentTime;

    float error = PhotoReflector.position / PhotoReflector.lightCount;  

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
  
    motor.motorDRV8833_R(RmotorSpeed);
    motor.motorDRV8833_L(LmotorSpeed);

  }

  if (s1 > threshold && s2 > threshold && s3 > threshold && s4 > threshold && s5 > threshold) {
    Serial.println("ライン外れ");
    motor.motorDRV8833_R(80);
    motor.motorDRV8833_L(80);
  }
}