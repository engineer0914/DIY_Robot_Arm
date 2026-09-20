//수동 제어 코드

#include <AccelStepper.h>//기본적인 헤더
#include <MultiStepper.h>//다중객체화시 포함 헤더

//boom
#define DIR0    5
#define STEP0   2
//arm
#define DIR01   6
#define STEP01  3
//base
#define DIR02   7
#define STEP02  4

//스위치 핀번호
#define xbuf    22
#define xbub    23
#define ybuf    24
#define ybub    25
#define zbuf    26
#define zbub    27
#define gbuf    28
#define gbub    29

//로봇팔 모터 순번 지정
#define robot_position_base 0
#define robot_position_boom 1
#define robot_position_arm  2

//모터별 설정
AccelStepper base = AccelStepper(1, STEP02, DIR02);//BASE
AccelStepper boom = AccelStepper(1, STEP0, DIR0);//BOOM
AccelStepper arm = AccelStepper(1, STEP01, DIR01);//ARM
MultiStepper steppers;

long positions[3];//위치 전역 변수 배열, 여기에 저장된 값이 모터별 위치 지정값이 됨
int modenum = 0,minval = 300, maxval = 700, midval = 500;//모드 선택 최소 최대 중앙값

void rotationmovementinsert(long base,long boom,long arm);//스텝수 입력에 따른 모터 동작, 0~200 입력
void anglemovementinset(double angle_base, double angle_boom, double angle_arm);//각도 입력에 따른 모터 동작 0~90 (권장)

void setup()
{
  //포트 세팅
  pinMode(DIR0,OUTPUT);
  pinMode(STEP0,OUTPUT);
  pinMode(DIR01,OUTPUT);
  pinMode(STEP01,OUTPUT);
  pinMode(DIR02,OUTPUT);
  pinMode(STEP02,OUTPUT);

  pinMode(xbuf,INPUT);
  pinMode(xbub,INPUT);
  pinMode(ybuf,INPUT);
  pinMode(ybub,INPUT);
  pinMode(zbuf,INPUT);
  pinMode(zbub,INPUT);
  pinMode(gbuf,INPUT);
  pinMode(gbub,INPUT);

  //시리얼 모듈 활성화
  Serial.begin(9600);

  //제한 최대 속도(임의로 지정)
  base.setMaxSpeed(3000);
  boom.setMaxSpeed(3000);
  arm.setMaxSpeed(3000);

  //내부 초기 포지션 변수 지정
  base.setCurrentPosition(0);
  boom.setCurrentPosition(0);
  arm.setCurrentPosition(0);

  //객체 선언 - 이거 순서 바뀌면 절대 안됨
  steppers.addStepper(base);
  steppers.addStepper(boom);
  steppers.addStepper(arm);

  delay(2000);
  Serial.print("\n initiate robotarm\n\n");
}

//base boom arm

void loop()
{
  for(;;) manualcontroller3();
  
//  for(;;)
//  {
//    //버튼에 따라 모드 숫자 변경
//    if(digitalRead(xbuf)!=1) modenum = 1;
//    if(digitalRead(xbub)!=1) modenum = 2;
//    if(digitalRead(ybuf)!=1) modenum = 3;
//    if(digitalRead(ybub)!=1) modenum = 4;
//    if(digitalRead(zbuf)!=1) modenum = 5;
//    if(digitalRead(zbub)!=1) modenum = 6;
//    if(modenum != 0) break;
//  }
//  if(modenum == 1)
//  {
//    for(;;) manualcontroller3();
//  }
//  else if(modenum == 2)
//  {
//    Serial.print("like this, you can add mode with modenum");
//    modenum = 0;
//  }
//  else{}

}

void manualcontroller3()//button 입력 수동 제어 함수
{
  int x,y,z,g,adder = 10, tstate = 0;//변수 선언 및 한번에 움직일 스텝수 지정
  //버튼 입력 지정
  int xf = digitalRead(xbuf);
  int xb = digitalRead(xbub);
  int yf = digitalRead(ybuf);
  int yb = digitalRead(ybub);
  int zf = digitalRead(zbuf);
  int zb = digitalRead(zbub);
  int gf = digitalRead(gbuf);
  int gb = digitalRead(gbub);

  if((xf == tstate)&&(xb != tstate)) x = maxval;//만약 파랑 버튼 두개의 상태가 다를때(=둘중 하나만 눌렸을때)
  else if((xf != tstate)&&(xb == tstate)) x = minval;//반대일때 x에 인가되는 값이 각각 다름
  else x = midval;//아예 입력이 안되었을때는 다른값을 줌

  if((yf == tstate)&&(yb != tstate)) y = maxval;
  else if((yf != tstate)&&(yb == tstate)) y = minval;
  else y = midval;

  if((zf == tstate)&&(zb != tstate)) z = maxval;
  else if((zf != tstate)&&(zb == tstate)) z = minval;
  else z = midval;

  if(x >= maxval) positions[robot_position_arm] -= adder;
  //만약 maxval이라는 값을 받을때는 adder값 만큼 스텝수에서 빼서 저장
  // = 어떤 방향으로 모터 동작
  else if(x <= minval) positions[robot_position_arm] += adder;
  //만약 minval이라는 값을 받을때는 adder값 만큼 스텝수에서 더해서 저장
  // = 어떤 방향과 반대되는 방향으로 모터 동작
  else{}//아무것도 안함

  if(y <= minval) positions[robot_position_boom] += adder;
  else if(y >= maxval) positions[robot_position_boom] -= adder;
  else{}

  if(z <= minval) positions[robot_position_base] += adder;
  else if(z >= maxval) positions[robot_position_base] -= adder;
  else{}

  steppers.moveTo(positions); //입력된 포지션 각 모터별 입력
  steppers.runSpeedToPosition(); //각각 동작
}
