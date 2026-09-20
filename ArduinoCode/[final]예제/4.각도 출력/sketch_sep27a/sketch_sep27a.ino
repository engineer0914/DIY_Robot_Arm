//구조
//https://www.youtube.com/watch?v=09Mpkjcr0bo&t=188s
//라이브러리
//https://www.airspayce.com/mikem/arduino/AccelStepper/classAccelStepper.html

//각도 출력 코드

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

//모터 회전스텝 및 기어비 계산
#define smallgearstep       200     //스텝모터의 제어방식 풀 하프 마이크로 중 풀스텝 기준 200스텝 인가시 1바퀴회전
#define microstep           16      //마이크로 스텝 비율
#define normgear            3.56    //32/9 = 3.5556 로봇팔에 32:9 기어 장착됨
#define planetarygear       5       //5:1의 감속기어가 추가적으로 부착되어있음

//모터별 설정
AccelStepper base = AccelStepper(1, STEP02, DIR02);//BASE
AccelStepper boom = AccelStepper(1, STEP0, DIR0);//BOOM
AccelStepper arm = AccelStepper(1, STEP01, DIR01);//ARM
MultiStepper steppers;

long positions[3];//위치 전역 변수 배열, 여기에 저장된 값이 모터별 위치 지정값이 됨
int modenum = 0,minval = 300, maxval = 700, midval = 500;//모드 선택 최소 최대 중앙값
double armangle_current, boomangle_current, baseangle_current;//현재 각도 전역 변수

void currentpositionshower();//오픈루프 제어상 현재 각도 출력
void rotationmovementinsert(long base,long boom,long arm);//스텝수 입력에 따른 모터 동작, 0~3200 입력(마이크로 스텝)
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
  base.setMaxSpeed(1200);
  boom.setMaxSpeed(1200);
  arm.setMaxSpeed(1200);

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
  anglemovementinset(0,30,0);//붐 30도 이동
  currentpositionshower();
  delay(1000);
  anglemovementinset(0,60,30);//암 붐 각각 30도 추가 이동
  currentpositionshower();
  delay(1000);
  anglemovementinset(0,90,90);//붐 암 90도 까지 동시 이동
  currentpositionshower();
  delay(1000);
  anglemovementinset(0,0,0);//원위치 복귀
  currentpositionshower();
  delay(1000);
}

void currentpositionshower()//현재 각도를 출력하는 함수
{
  long instantshower;//지역변수

  instantshower = base.currentPosition();//프로그램 내부적으로 베이스 모터의 현재 스텝 값을 읽어옴
  baseangle_current = (double)instantshower*360/(smallgearstep*microstep*normgear);//기어비 계산해서 전역변수에 각도로 저장
  Serial.print("\nangle current base:");
  Serial.println(baseangle_current);//각도 출력

  instantshower = boom.currentPosition();
  boomangle_current = (double)instantshower*360/(smallgearstep*microstep*planetarygear*normgear);//감속기어가 부착되어 수식 변형
  Serial.print("angle current boom:");
  Serial.println(boomangle_current);

  instantshower = arm.currentPosition();
  armangle_current = (double)instantshower*360/(smallgearstep*microstep*planetarygear*normgear);
  Serial.print("angle current arm:");
  Serial.println(armangle_current);
}

void rotationmovementinsert(long base,long boom,long arm)
{
  //0~200사이 값을 입력 받아 스텝값으로 저장
  positions[robot_position_base] = base;
  positions[robot_position_arm] = arm;
  positions[robot_position_boom] = boom;
  //동작
  steppers.moveTo(positions); //포지션 입력
  steppers.runSpeedToPosition(); //각각 동작
}

void anglemovementinset(double angle_base, double angle_boom, double angle_arm)
{
  double instantbo, instantba, instantar;//지역 변수 선언
  //베이스의 경우 감속기어가 없이 평기어만 부착되어있음에 원하는 각도에 따라 요구되는 스텝수를 계산하는 수식은 다음과 같다.
  //1.스텝모터 1회전(360도)시 요구되는 스텝 = 200, (1스텝당 1.8도 이동)
  //2.마이크로 스텝 설정시, 스텝 2,4,8,16배 증폭가능, 16배 증폭에 따라 스텝모터 1회전시 요구되는 스텝 = 200*16
  //3.평기어 기어비 = 32/9(3.56)임에따라 평기어 1회전시 요구되는 스텝 = 200*16*3.56
  //4.원하는 평기어 각도 입력시 이에 상응하는 스텝수 계산식 = (angle/360)*200*16*3.56
  instantba = (angle_base*smallgearstep*microstep*normgear/360);
  //이외 모터의 경우 감속기어가 추가적으로 부착되어 있음에 수정된 수식은 다음과 같다.
  //4.위의 수식에서 추가적으로 5:1감속기어가 부착되어, 감속기어 1회전에 요구되는 스텝수 = 200*16*3.56*5
  //5.원하는 감속기어 각도 입력시 이에 상응하는 스텝수 계산식 = (angle/360)*200*16*3.56*5
  instantbo = (planetarygear*angle_boom*smallgearstep*microstep*normgear/360);
  instantar = (planetarygear*angle_arm*smallgearstep*microstep*normgear/360);
  //계산된 스텝값 인자로 입력, 이때, 스텝수는 long 형이고 현재는 세밀한 계산을 위해 double 형이므로 '형변환' 시행 및 입력
  rotationmovementinsert((long)instantba, (long)instantbo, (long)instantar);
}
