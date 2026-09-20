/*
아두이노 cnc쉴드 써서 제작 xyz라고 쓰여진데 있어서 하나 다른데만 안넣으면됨
z가 베이스
y가 링크1 암
x가 링크2 붐
28byj모터 그리퍼 사용(쉴드내 z 11 y 10 x 9 spnen 12 을 각각 uln에 연결해서 씀)
a4988 드라이버 사용
핀맵은 하단부에 적어둠
동작에 앞서 led를 켜고 끄게 해둠
accelstepper, stepper 라이브러리 같이쓰기
8번핀 enable gnd 동작이라서 스위치 연결하면 전원 항시 연결에 동작 활성화 비활성화 할수있음
a4a5에 조이스틱 연결
d11에 스위치 연결
*/

/*
cnc구조 관련 링크
http://john-home.iptime.org:8085/xe/index.php?mid=board_jalN13&document_srl=1060
*/

/*
마이크로 스텝
분주비 16:1
감속기어 비율 5:1
작은 기어 회전에 걸리는 풀 스텝 수 200
큰기어와 작은 기어간 기어비 9:32 - 711
*/

/*
  목적 : 붐암 컨트롤러, 속도 및 상태 조정
*/

/*
AccelStepper stepper = AccelStepper(연결방식, STEP핀, DIR핀);
stepper.setMaxSpeed(1000); //최대속도
stepper.setCurrentPosition(0); //현재 스탭을 설정
stepper.currentPosition(); //현재 스탭을 반환
stepper.setSpeed(200); //스탭모터의 방향과 속도를 설정
stepper.runSpeed(); //스탭모터 작동(고정속도)
stepper.setAcceleration(2000); //가속량을 설정함
stepper.moveTo(1000); //목표스탭량을 설정함
stepper.runToPosition(); //가속도를 붙혀서 스탭모터제어
*/

#include <AccelStepper.h>

#define xval A4
#define yval A5
#define sw 10
#define en 8
#define DIR0 5
#define STEP0 2
#define DIR01 6
#define STEP01 3
#define DIR02 7
#define STEP02 4
#define led 13

const int stepsPerRevolution = 2048; //2048 = 1바퀴

AccelStepper arm = AccelStepper(1, STEP0, DIR0);//링크2번
AccelStepper boom = AccelStepper(1, STEP01, DIR01);//링크1번
AccelStepper base = AccelStepper(1, STEP02, DIR02);//베이스회전

void setup()
{
  pinMode(DIR0,OUTPUT);
  pinMode(STEP0,OUTPUT);
  pinMode(DIR01,OUTPUT);
  pinMode(STEP01,OUTPUT);
  pinMode(DIR02,OUTPUT);
  pinMode(STEP02,OUTPUT);
  pinMode(led,OUTPUT);
  pinMode(sw,INPUT);
  pinMode(xval,INPUT);
  pinMode(yval,INPUT);

  boom.setMaxSpeed(10000);
  boom.setCurrentPosition(0);
  arm.setMaxSpeed(10000);
  arm.setCurrentPosition(0);
  base.setMaxSpeed(10000);
  base.setCurrentPosition(0);
  
  Serial.begin(57600);
  Serial.print("initiate\n");
}

int motor_speed_min = 100, motor_speed_max = 10000, motor_speed_offset = 3000;
int rate = 16, circle = 711, circle_small = 200, rate_gear = 1;
int aimfor = rate_gear*rate*circle/10;
int minval = 350, maxval = 600, swstate = 0, dirset_arm = 0, dirset_boom = 0,dirset_base = 0;
long position_arm = 0,position_boom = 0,position_base = 0;

void ledon();
void ledoff();
void baseleft();
void baseright();
void armdown();
void armup();
void boomdown();
void boomup();

void armturn();
void boomturn();
void baseturn();
void speedset();

void loop()
{
  speedset();

  int x = analogRead(xval);
  int y = analogRead(yval);
  
  if(x >= maxval)
  {
    dirset_boom = 1;
    position_boom += 1;
    Serial.print("BOOM_UP\n");
    boomturn();
  }

  else if(x <= minval)
  {
    dirset_boom = -1;
    position_boom -= 1;
    Serial.print("BOOM_DOWN\n");
    boomturn();
  }
  else{}
  
  if(y <= minval)
  {
    dirset_arm = 1;
    position_arm += 1;
    Serial.print("ARM_FOWARD\n");
    armturn();
  }
  
  else if(y >= maxval)
  {
    dirset_arm = -1;
    position_arm -= 1;
    Serial.print("ARM_BACKWARD\n");
    armturn();
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


void speedset()
{
  if(digitalRead(sw)==0)
  {
  swstate=swstate+1;
  delay(200);
  Serial.println(motor_speed_offset);
  }

  if(swstate == 4)swstate = 0;
  if(swstate == 0)motor_speed_offset = 1000;
  else if(swstate == 1)motor_speed_offset = 2000;
  else if(swstate == 2)motor_speed_offset = 3500;
  else if(swstate == 3)motor_speed_offset = 5000;
  else{}
}



   //#########################################################################################

void baseleft()
{
  
  ledon();
  
  base.setSpeed(motor_speed_offset);
    
    while(base.currentPosition() != rate*circle*0.25)
    {
      base.runSpeed();
    }
    
  base.setSpeed(0); //멈춤!
    
  ledoff();

}

void baseright() 
{
  ledon();
  
  base.setSpeed(-1*motor_speed_offset);
  
    while(base.currentPosition() != 0)
    {
      base.runSpeed();
    }
    
  base.setSpeed(0); //멈춤!

  ledoff();
}

void armdown() 
{
  ledon();
  
  arm.setSpeed(motor_speed_offset);

    while(arm.currentPosition() != aimfor)
    {
      arm.runSpeed();
    }

  arm.setSpeed(0); //멈춤!

  ledoff();
}

void armup() 
{
  ledon();
  
  arm.setSpeed(-1*motor_speed_offset);
  
  while(arm.currentPosition() != 0)
  {
    arm.runSpeed();
  }
    
    arm.setSpeed(0); //멈춤!
    ledoff();
}

void boomdown() 
{
  ledon();
  boom.setSpeed(-1*motor_speed_offset);
  while(boom.currentPosition() != 0)
  {
    boom.runSpeed();
  }
    boom.setSpeed(0); //멈춤!
    ledoff();
}

void boomup()
{
  ledon();
  boom.setSpeed(motor_speed_offset);
    
    while(boom.currentPosition() != aimfor)
    {
      boom.runSpeed();
    }
    boom.setSpeed(0); //멈춤!
    ledoff();
}

void ledon()

{
  digitalWrite(led,HIGH);
}

void ledoff()
{
  digitalWrite(led,LOW);
}
