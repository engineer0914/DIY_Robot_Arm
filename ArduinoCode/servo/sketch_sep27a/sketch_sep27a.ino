#include <Servo.h> // 서보모터 라이브러리 포함

Servo myServo;  // 서보모터 객체 생성

void setup() {
  myServo.attach(9); // 서보모터 신호선을 9번 핀에 연결
}

void loop() {
  // 0도로 이동
  myServo.write(0);
  delay(1000); // 모터가 움직일 시간을 줌 (1초 대기)

  // 90도로 이동 (중간)
  myServo.write(90);
  delay(1000);

  // 180도로 이동 (끝)
  myServo.write(180);
  delay(1000);
}
