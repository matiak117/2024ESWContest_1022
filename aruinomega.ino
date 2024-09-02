// 모터 제어 핀 설정
const int leftMotorPWM1 = 3;
const int leftMotorDir1 = 2;
const int rightMotorPWM1 = 8;
const int rightMotorDir1 = 9;

const int leftMotorPWM2 = 5;
const int leftMotorDir2 = 4;
const int rightMotorPWM2 = 10;
const int rightMotorDir2 = 11;

const int leftMotorPWM3 = 7;
const int leftMotorDir3 = 6;
const int rightMotorPWM3 = 12;
const int rightMotorDir3 = 13;

// 초음파 센서 핀 설정
const int TRIG1 = A0;
const int ECHO1 = A1;
const int TRIG2 = A2;
const int ECHO2 = A3;
const int TRIG3 = A4;
const int ECHO3 = A5;
const int TRIG4 = A6;
const int ECHO4 = A7;

void setup() {
  // 시리얼 통신 시작
  Serial.begin(9600);
  
  // 모터 제어 핀을 출력으로 설정
  pinMode(leftMotorPWM1, OUTPUT);
  pinMode(leftMotorDir1, OUTPUT);
  pinMode(rightMotorPWM1, OUTPUT);
  pinMode(rightMotorDir1, OUTPUT);
  
  pinMode(leftMotorPWM2, OUTPUT);
  pinMode(leftMotorDir2, OUTPUT);
  pinMode(rightMotorPWM2, OUTPUT);
  pinMode(rightMotorDir2, OUTPUT);
  
  pinMode(leftMotorPWM3, OUTPUT);
  pinMode(leftMotorDir3, OUTPUT);
  pinMode(rightMotorPWM3, OUTPUT);
  pinMode(rightMotorDir3, OUTPUT);
  
  // 초음파 센서 핀 모드 설정
  pinMode(TRIG1, OUTPUT);
  pinMode(ECHO1, INPUT);
  pinMode(TRIG2, OUTPUT);
  pinMode(ECHO2, INPUT);
  pinMode(TRIG3, OUTPUT);
  pinMode(ECHO3, INPUT);
  pinMode(TRIG4, OUTPUT);
  pinMode(ECHO4, INPUT);
}

void loop() {
  long duration1, distance1;
  long duration2, distance2;
  long duration3, distance3;
  long duration4, distance4;

  // 초음파 센서 1로 거리 측정
  distance1 = measureDistance(TRIG1, ECHO1);
  
  // 초음파 센서 2로 거리 측정
  distance2 = measureDistance(TRIG2, ECHO2);
  
  // 초음파 센서 3로 거리 측정
  distance3 = measureDistance(TRIG3, ECHO3);
  
  // 초음파 센서 4로 거리 측정
  distance4 = measureDistance(TRIG4, ECHO4);

  // 거리 값 시리얼 모니터에 출력
  Serial.print("Distance1: ");
  Serial.print(distance1);
  Serial.print(" cm, Distance2: ");
  Serial.print(distance2);
  Serial.print(" cm, Distance3: ");
  Serial.print(distance3);
  Serial.print(" cm, Distance4: ");
  Serial.print(distance4);
  Serial.println(" cm");

  // 거리 값이 20cm 이하일 경우 모터 긴급 정지
  if (distance1 <= 20 || distance2 <= 20 || distance3 <= 20 || distance4 <= 20) {
    stopMotors();
  } else {
    // 시리얼 데이터가 있는지 확인
    if (Serial.available() > 0) {
      // 시리얼 데이터 읽기
      String data = Serial.readStringUntil('\n');
      // 데이터 파싱
      parseAndSetMotorSpeeds(data);
    }
  }

  delay(100); // 0.1초 대기
}

long measureDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH);
  return duration * 17 / 1000;
}

void parseAndSetMotorSpeeds(String data) {
  int commaIndex1 = data.indexOf(',');
  if (commaIndex1 > 0) {
    String leftSpeedStr1 = data.substring(0, commaIndex1);
    String remainingData1 = data.substring(commaIndex1 + 1);
    
    int commaIndex2 = remainingData1.indexOf(',');
    if (commaIndex2 > 0) {
      String rightSpeedStr1 = remainingData1.substring(0, commaIndex2);
      String remainingData2 = remainingData1.substring(commaIndex2 + 1);

      int commaIndex3 = remainingData2.indexOf(',');
      if (commaIndex3 > 0) {
        String leftSpeedStr2 = remainingData2.substring(0, commaIndex3);
        String remainingData3 = remainingData2.substring(commaIndex3 + 1);
        
        int commaIndex4 = remainingData3.indexOf(',');
        if (commaIndex4 > 0) {
          String rightSpeedStr2 = remainingData3.substring(0, commaIndex4);
          String remainingData4 = remainingData3.substring(commaIndex4 + 1);
          
          int commaIndex5 = remainingData4.indexOf(',');
          if (commaIndex5 > 0) {
            String leftSpeedStr3 = remainingData4.substring(0, commaIndex5);
            String rightSpeedStr3 = remainingData4.substring(commaIndex5 + 1);
            
            // 문자열을 부동 소수점 수로 변환
            float leftSpeed1 = atof(leftSpeedStr1.c_str());
            float rightSpeed1 = atof(rightSpeedStr1.c_str());
            float leftSpeed2 = atof(leftSpeedStr2.c_str());
            float rightSpeed2 = atof(rightSpeedStr2.c_str());
            float leftSpeed3 = atof(leftSpeedStr3.c_str());
            float rightSpeed3 = atof(rightSpeedStr3.c_str());
            
            // 모터 속도 및 방향 설정
            setMotorSpeed(leftMotorPWM1, leftMotorDir1, leftSpeed1);
            setMotorSpeed(rightMotorPWM1, rightMotorDir1, rightSpeed1);
            setMotorSpeed(leftMotorPWM2, leftMotorDir2, leftSpeed2);
            setMotorSpeed(rightMotorPWM2, rightMotorDir2, rightSpeed2);
            setMotorSpeed(leftMotorPWM3, leftMotorDir3, leftSpeed3);
            setMotorSpeed(rightMotorPWM3, rightMotorDir3, rightSpeed3);
          }
        }
      }
    }
  }
}
void setMotorSpeed(int pwmPin, int dirPin, float speed) {
  int maxPWMValue = 155; // 최대 PWM 값 설정 (255의 절반으로 속도를 줄임)
  
  if (speed >= 0) {
    digitalWrite(dirPin, HIGH);  // 전진
  } else {
    digitalWrite(dirPin, LOW);   // 후진
    speed = -speed;              // 속도를 양수로 변환
  }
  
  // 속도를 조정하여 최대 PWM 값으로 설정
  int pwmValue = int(speed * maxPWMValue);
  
  // PWM 값을 0과 maxPWMValue 사이로 제한
  pwmValue = constrain(pwmValue, 0, maxPWMValue);
  
  analogWrite(pwmPin, pwmValue);
}

void stopMotors() {
  // 모든 모터의 PWM 출력을 0으로 설정하여 모터를 정지시킵니다.
  analogWrite(leftMotorPWM1, 0);
  analogWrite(rightMotorPWM1, 0);
  analogWrite(leftMotorPWM2, 0);
  analogWrite(rightMotorPWM2, 0);
  analogWrite(leftMotorPWM3, 0);
  analogWrite(rightMotorPWM3, 0);
  
  // 방향 핀도 설정하여 모터의 방향을 멈추게 합니다.
  digitalWrite(leftMotorDir1, LOW);
  digitalWrite(rightMotorDir1, LOW);
  digitalWrite(leftMotorDir2, LOW);
  digitalWrite(rightMotorDir2, LOW);
  digitalWrite(leftMotorDir3, LOW);
  digitalWrite(rightMotorDir3, LOW);

  // 시리얼 모니터에 긴급 정지 메시지를 출력합니다.
  Serial.println("Emergency Stop!");
}
