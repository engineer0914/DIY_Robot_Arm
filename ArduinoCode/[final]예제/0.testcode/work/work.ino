/*
 * Incheon National University
 * Project:    PARALLEL ROBOT ARM CONTROL
 * Laboratory: Robotics & Energy Convergence Lab
 * Date:       2024 FALL
 * Professor:  Wooyong Kim
 * Assistant:  Dongsik Park
 *
 * 목적 - 버튼 입력 확인후 축동작 확인
 * 
 * 사용방법
 * 
 * 1.아두이노에서 "AccelStepper" 라는 라이브러리를 설치 - https://rasino.tistory.com/344
 *   "3-1 라이브러리 매니저를 통하여 라이브러리 검색후 설치하는 방법" 참고
 *   
 * 2.버튼 입력을 동기화 하려면, ELECTRICAL MANUAL을 통해서 8버튼 연결파트 참조하여 재결선 할 것
 *   선택적인 사항이며, 미진행시 인식 버튼이 바뀜
 *   
 * 3.아두이노 코드 전송 및 긴급정지 버튼 해제
 * 
 * 4.시리얼 모니터 열기(Ctrl + Shift + M), Baud rate 9600 설정
 * 
 * 5.아무 버튼 10번 누르기, 누른 버튼에 따라 할당된 번호가 출력
 * 
 * 6.10번의 입력 이후, 버튼에 따라 붐,암,베이스 축에 해당되는 스텝 모터 회전 확인 가능.
*/

#include <AccelStepper.h>

#define ON  1
#define OFF 0

#define xbuf    22
#define xbub    23
#define ybuf    24
#define ybub    25
#define zbuf    26
#define zbub    27
#define gbuf    28
#define gbub    29

#define DIR0 5
#define STEP0 2
#define DIR01 6
#define STEP01 3
#define DIR02 7
#define STEP02 4
#define led 13

//const int stepsPerRevolution = 2048; //2048 = 1바퀴

AccelStepper boom = AccelStepper(1, STEP0, DIR0);//링크2번
AccelStepper arm = AccelStepper(1, STEP01, DIR01);//링크1번
AccelStepper base = AccelStepper(1, STEP02, DIR02);//베이스회전

void setup()
{
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

  boom.setMaxSpeed(10000);
  boom.setCurrentPosition(0);
  arm.setMaxSpeed(10000);
  arm.setCurrentPosition(0);
  base.setMaxSpeed(1000);
  base.setCurrentPosition(0);
  
  Serial.begin(9600);
  Serial.print("initiate\n");
}

int motor_speed_offset = 5000;
int dirset_arm = 0, dirset_boom = 0,dirset_base = 0;
long position_arm = 0,position_boom = 0,position_base = 0;
int modenum = 0,minval = 300, maxval = 700, midval = 500;//조이스틱 ADC 값 각각 최소 최대 중앙 값, 모드 선택

void perfomance_check(void);
void armturn(void);
void boomturn(void);
void baseturn(void);

void loop()
{
  perfomance_check();
  Serial.println();
  Serial.print("initiate robotArm manual mode \n press reset to exit manual mode and reset robotArm");
  Serial.println();
  Serial.print("button controller enable:");
  for(;;) manualcontroller3();
}

void perfomance_check()
{
  int counter = 0, ttstate = ON;
  Serial.print("\n initiate robotArm function checking mode, push button\n");  
  for(;;)
  {
    for(;;)
    {
      if(digitalRead(xbuf)!=ttstate) modenum = 1;
      if(digitalRead(xbub)!=ttstate) modenum = 2;
      if(digitalRead(ybuf)!=ttstate) modenum = 3;
      if(digitalRead(ybub)!=ttstate) modenum = 4;
      if(digitalRead(zbuf)!=ttstate) modenum = 5;
      if(digitalRead(zbub)!=ttstate) modenum = 6;
      if(digitalRead(gbuf)!=ttstate) modenum = 7;
      if(digitalRead(gbub)!=ttstate) modenum = 8;
      if(modenum != 0) break;
    }
    Serial.println(modenum);
    modenum = 0;
    delay(500);
    counter++;
    if(counter == 10) break;
  }

  Serial.print("\n done");
}

void manualcontroller3()//button
{
  int x,y,z,adder = 100,tstate = OFF;//tstate means enable threshhold
  int xf = digitalRead(xbuf);
  int xb = digitalRead(xbub);
  int yf = digitalRead(ybuf);
  int yb = digitalRead(ybub);
  int zf = digitalRead(zbuf);
  int zb = digitalRead(zbub);

  if((xf == tstate)&&(xb != tstate)) x = maxval;
  else if((xf != tstate)&&(xb == tstate)) x = minval;
  else x = midval;

  if((yf == tstate)&&(yb != tstate)) y = maxval;
  else if((yf != tstate)&&(yb == tstate)) y = minval;
  else y = midval;

  if((zf == tstate)&&(zb != tstate)) z = maxval;
  else if((zf != tstate)&&(zb == tstate)) z = minval;
  else z = midval;

  if(y < midval)
  {
    dirset_boom = 1;
    position_boom += adder;
    Serial.print("BOOM_DOWN\n");
    boomturn();
  }

  else if(y > midval)
  {
    dirset_boom = -1;
    position_boom -= adder;
    Serial.print("BOOM_UP\n");
    boomturn();
  }
  else{}
  
  if(x < midval)
  {
    dirset_arm = 1;
    position_arm += adder;
    Serial.print("ARM_BACKWARD\n");
    armturn();
  }
  else if(x > midval)
  {
    dirset_arm = -1;
    position_arm -= adder;
    Serial.print("ARM_FOWARD\n");
    armturn();
  }
  else{}

  if(z < midval)
  {
    dirset_base = 1;
    position_base += adder;
    Serial.print("BASE_FOWARD\n");
    baseturn();
  }
  else if(z > midval)
  {
    dirset_base = -1;
    position_base -= adder;
    Serial.print("BASE_BACKWARD\n");
    baseturn();
  }
  else{}
}

void armturn()
{
  arm.setSpeed(dirset_arm*motor_speed_offset);
  while(1)
  {
    arm.runSpeed();
    if(arm.currentPosition() == position_arm) break;
  }
  arm.setSpeed(0); //멈춤!
}

void boomturn()
{
  boom.setSpeed(dirset_boom*motor_speed_offset);
  while(1)
  {
    boom.runSpeed();
    if(boom.currentPosition() == position_boom) break;
  }
  boom.setSpeed(0); //멈춤!
}

void baseturn()
{
  base.setSpeed(dirset_base*motor_speed_offset);
  while(1)
  {
    base.runSpeed();
    if(base.currentPosition() == position_base) break;
  }
  base.setSpeed(0); //멈춤!
}
