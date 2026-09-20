#include <AccelStepper.h>
#include <MultiStepper.h>

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

//boom
#define DIR0    5
#define STEP0   2
//arm
#define DIR01   6
#define STEP01  3
//base
#define DIR02   7
#define STEP02  4

#define robot_position_base 0
#define robot_position_boom 1
#define robot_position_arm  2

//기어비 선언
#define smallgearstep       200     //one circle for step
#define microstep           16      //microstep
#define normgear            3.56    //32/9 = 3.5556
#define planetarygear       5       //5:1

//설명
#define description         ON      //description 0 off 1 on

AccelStepper base = AccelStepper(1, STEP02, DIR02);//BASE
AccelStepper boom = AccelStepper(1, STEP0, DIR0);//BOOM
AccelStepper arm = AccelStepper(1, STEP01, DIR01);//ARM

MultiStepper steppers;

long positions[3];//위치 전역 변수 배열, 여기에 저장된 값이 모터별 위치 지정값이 됨
int modenum = 0,minval = 300, maxval = 700, midval = 500;//조이스틱 ADC 값 각각 최소 최대 중앙 값, 모드 선택
float speedmotor = 1200.0;//제한 최대 속도
double armangle_current, boomangle_current, baseangle_current;//현재 각도 전역 변수
double out_boomangle, out_armangle;//역기구학 출력 각도 전역 변수
double manual_x=15 , manual_y = 24;//수동좌표 시작 상태 변수, [cm]
double posx = 12.0, posy = 12.0, posz=0.0, y_offset=12;//xy 평면 좌표, z 회전 각도, 로봇 붐 관절 중앙 지면 사이 길이
double boom_L1 = 12.0;//boom의 길이
double arm_L3 = 12.0;//arm의 길이
double endeffectoroffset_wd = 3.0;//arm의 끝과 엔드이펙터 사이의 거리

void perfomance_1();//한붓그리기
void manualcontroller3();//매뉴얼 버튼 컨트롤러 모드
void rotationmovementinsert(long base,long boom,long arm);//스텝 회전수 입력 동작
void anglemovementinset(double angle_base, double angle_boom, double angle_arm);//각도 입력 동작
void currentpositionshower();//현재 위치 각도 산출 함수
void Inverse_Kinematics_Calculator(double P_x, double P_y);//역기구학 위치 계산 입력:좌표, 출력:각도
void Inverse_Kinematics_Action(double R_z,double Pos_x, double Pos_y);//역기구학 출력

void setup()
{    
  //포트 세팅
  pinMode(DIR0,OUTPUT);
  pinMode(STEP0,OUTPUT);
  pinMode(DIR01,OUTPUT);
  pinMode(STEP01,OUTPUT);
  pinMode(DIR02,OUTPUT);
  pinMode(STEP02,OUTPUT);

  //시리얼 모듈 활성화
  Serial.begin(9600);

  pinMode(xbuf,INPUT);
  pinMode(xbub,INPUT);
  pinMode(ybuf,INPUT);
  pinMode(ybub,INPUT);
  pinMode(zbuf,INPUT);
  pinMode(zbub,INPUT);
  pinMode(gbuf,INPUT);
  pinMode(gbub,INPUT);

  Serial.print("\ninitiate robotarm\n\n");

  //제한 최대 속도
  speedmotor = 1200.0;
  base.setMaxSpeed(speedmotor);
  if(planetarygear==5) speedmotor = 8000.0;
  boom.setMaxSpeed(speedmotor);
  arm.setMaxSpeed(speedmotor);

  //내부 초기 포지션 변수 오픈 루프 제어
  base.setCurrentPosition(0);
  boom.setCurrentPosition(0);
  arm.setCurrentPosition(0);

  //객체 선언 - 이거 순서 바뀌면 절대 안됨
  steppers.addStepper(base);
  steppers.addStepper(boom);
  steppers.addStepper(arm);
  
  delay(2000);
}

//base boom arm

void loop()
{
  //모드 선택
  for(;;)
  {
    if(digitalRead(xbuf)!=1) modenum = 1;
    if(digitalRead(xbub)!=1) modenum = 2;
    if(digitalRead(ybuf)!=1) modenum = 3;
    if(digitalRead(ybub)!=1) modenum = 4;
    if(digitalRead(zbuf)!=1) modenum = 5;
    if(digitalRead(zbub)!=1) modenum = 6;
    if(modenum != 0) break;
  }

  if(modenum == 1)//수동제어
  {
    Serial.println();
    Serial.print("\ninitiate robotArm manual mode \n press reset to exit manual mode and reset robotArm");
    Serial.println();
    Serial.print("button controller enable:");
    for(;;) manualcontroller3();
  }
  else if(modenum == 2)//시리얼 역기구학 제어
  {
    Serial.println();
    Serial.print("\ninitiate robotArm manual mode \n press reset to exit manual mode and reset robotArm");
    Serial.println();
    Serial.print("serial controller enable:");
    for(;;)
    {
      if(Serial.available())//시리얼 통신 활성화시
      {
        switch(Serial.read())//시리얼 통신으로 문자입력 받았을때
        {
          //wasd로 엔드이펙터를 xy축 이동
          case'w': manual_y++; break;
          case's': manual_y--; break;
          case'a': manual_x--; break;
          case'd': manual_x++; break;
          default: break;//default nonmovement
        }
        Inverse_Kinematics_Action(0,manual_x, manual_y);//수동 제어후 좌표 입력
      }
    }
  }
  else if(modenum == 3)//정기구학 각도 기준 계산된 좌표 이동
  {
    Inverse_Kinematics_Action(0,13.39,18);//붐 0도 암 30도
    currentpositionshower();
    delay(5000);
    Inverse_Kinematics_Action(0,23.78,12);//붐 60도 암 30도
    currentpositionshower();
    delay(5000);
    Inverse_Kinematics_Action(0,15,24);//붐 0도 암 0도
    currentpositionshower();
    delay(5000);
  }
  else if(modenum == 4)//한붓 그리기
  {
    for(;;) perfomance_1();
  }
  else{}
  modenum = 0;//모드 초기화
}

void perfomance_1()//한붓 그리기
{
  int j = 1;//변수값 증가시 지연 시간 증가

/*
 * 마름모 그리기
15,10 20,15
20,15 15,20
15,20 10,15
10,15 15,10
*/

  for(double i=24;i>=20;i--) Inverse_Kinematics_Action(0,15,i);
  delay(100*j);

  for(double i=0;i<=5;i++) Inverse_Kinematics_Action(0,15-i,20-i);
  delay(100*j);

  for(double i=0;i<=5;i++) Inverse_Kinematics_Action(0,10+i,15-i);
  delay(100*j);
  
  for(double i=0;i<=5;i++) Inverse_Kinematics_Action(0,15+i,10+i);
  delay(100*j);

  for(double i=0;i<=5;i++) Inverse_Kinematics_Action(0,20-i,15+i);
  delay(100*j);



/*
 * 사각형 그리기
15,20 20,20
20,20 20,10
20,10 10,10
10,10 10,20
10,20 15,20
*/

  for(int i=15;i<=20;i++) Inverse_Kinematics_Action(0,(double)i,20);
  delay(100*j);

  for(int i=20;i>=10;i--) Inverse_Kinematics_Action(0,20,(double)i);
  delay(100*j);

  for(int i=20;i>=10;i--) Inverse_Kinematics_Action(0,(double)i,10);
  delay(100*j);

  for(int i=10;i<=20;i++) Inverse_Kinematics_Action(0,10,(double)i);
  delay(100*j);

  for(int i=10;i>=15;i--) Inverse_Kinematics_Action(0,(double)i,20);
  delay(100*j);

  for(int i=10;i<=15;i++) Inverse_Kinematics_Action(0,(double)i,20);
  delay(100*j);

  Inverse_Kinematics_Action(0,15,24);

  delay(5000);
}

void Inverse_Kinematics_Action(double R_z,double Pos_x, double Pos_y)
{
  if(description == ON)
  {
    //입력 사항 출력
    Serial.println();
    Serial.print("input base angle:");
    Serial.println(R_z);
    Serial.print("input Pos_x:");
    Serial.println(Pos_x);
    Serial.print("input Pos_y:");
    Serial.println(Pos_y);
  }
  Inverse_Kinematics_Calculator(Pos_x,Pos_y);//역기구학 계산기
  anglemovementinset(R_z,out_boomangle,out_armangle);//계산후 각도 이동
}

void Inverse_Kinematics_Calculator(double P_x, double P_y)
{
  //double boom_L1 = 12.0;//boom의 길이
  //double arm_L3 = 12.0;//arm의 길이
  //double endeffectoroffset_wd = 3.0;//arm의 끝과 엔드이펙터 사이의 거리
  
  double boomangle_th1, armangle_th2, th21;//붐 각도, 암 각도, 붐암 사잇각
  P_y -= y_offset;//오프셋 제거
  double D = sqrt(pow((P_x) - endeffectoroffset_wd, 2) + pow(P_y, 2));//엔드이펙터 관절의 3차원 공간 위치
  //double posD = sqrt(pow(P_x, 2) + pow(P_y, 2));//엔드이펙터의 3차원 공간 위치
  //각도단위:라디안

  //th21값 계산(cos 2법칙 사용)
  th21 = acos((pow(boom_L1, 2) + pow(arm_L3, 2) - pow(D, 2)) / (2 * boom_L1 * arm_L3));
  //th1을 계산하기 위해 삼각형 p0,p1,p4에서 각p1-p0-p4의 각도값 계산
  double alpha = acos((pow(boom_L1, 2) + pow(D, 2) - pow(arm_L3, 2)) / (2 * boom_L1 * D));
  //th1을 계산하기 위해 삼각형 p0,p4_y,p4_x에서 각p4_y-p0-p4_x의 각도값 계산
  double psi = acos((pow(D, 2) + pow(P_x - endeffectoroffset_wd, 2) - pow(P_y, 2)) / (2 * D * (P_x - endeffectoroffset_wd)));

  //라디안->도 변환
  alpha = alpha * 180 / PI;
  psi = psi * 180 / PI;
  th21 = th21 * 180 / PI;//붐 암 사이 각

  if (P_y >= 0) //만약 엔드이펙터의 높이가 boom,arm 모터의 중심보다 위에 있을경우
    boomangle_th1 = alpha + psi; // 붐 각도
  else//만약 엔드이펙터의 높이가 boom,arm 모터의 중심보다 아래에 있을경우
    boomangle_th1 = alpha - psi; //붐 각도

  armangle_th2 = th21 + boomangle_th1; //암 모터 각도

  out_boomangle = (-1*boomangle_th1)+90;//오프셋 제외 최종 각도 붐
  out_armangle = (-1*armangle_th2)+180;//오프셋 제외 최종 각도 암

  if(description == ON)
  {
    Serial.print("D:");
    Serial.println(D);
    Serial.print("between boom & arm(th21):");
    Serial.println(th21);
    Serial.print("boom angle(th1):");
    Serial.println(out_boomangle);
    Serial.print("arm angle(th2):");
    Serial.println(out_armangle);
    delay(5000);
  }
}

void rotationmovementinsert(long base,long boom,long arm)
{
  positions[robot_position_base] = base;
  positions[robot_position_arm] = arm;
  positions[robot_position_boom] = boom;
  steppers.moveTo(positions); //포지션 입력
  steppers.runSpeedToPosition(); //각각 동작
}

void anglemovementinset(double angle_base, double angle_boom, double angle_arm)
{
  double instantbo, instantba, instantar;
  instantba = (angle_base*smallgearstep*microstep*normgear/360);
  instantbo = (planetarygear*angle_boom*smallgearstep*microstep*normgear/360);
  instantar = (planetarygear*angle_arm*smallgearstep*microstep*normgear/360);
  rotationmovementinsert((long)instantba, (long)instantbo, (long)instantar);
}

void currentpositionshower()
{
  long instantshower;

  instantshower = base.currentPosition();
  baseangle_current = (double)instantshower*360/(smallgearstep*microstep*normgear);
  Serial.print("\nangle current base:");
  Serial.println(baseangle_current);
  
  instantshower = boom.currentPosition();
  boomangle_current = (double)instantshower*360/(smallgearstep*microstep*planetarygear*normgear);
  Serial.print("angle current boom:");
  Serial.println(boomangle_current);

  instantshower = arm.currentPosition();
  armangle_current = (double)instantshower*360/(smallgearstep*microstep*planetarygear*normgear);
  Serial.print("angle current arm:");
  Serial.println(armangle_current);
}

void manualcontroller3()//button
{
  int x,y,z,g,adder = 10, tstate = OFF;//tstate means enable threshhold
  int xf = digitalRead(xbuf);
  int xb = digitalRead(xbub);
  int yf = digitalRead(ybuf);
  int yb = digitalRead(ybub);
  int zf = digitalRead(zbuf);
  int zb = digitalRead(zbub);
  int gf = digitalRead(gbuf);
  int gb = digitalRead(gbub);

  if((xf == tstate)&&(xb != tstate)) x = maxval;
  else if((xf != tstate)&&(xb == tstate)) x = minval;
  else x = midval;

  if((yf == tstate)&&(yb != tstate)) y = maxval;
  else if((yf != tstate)&&(yb == tstate)) y = minval;
  else y = midval;

  if((zf == tstate)&&(zb != tstate)) z = maxval;
  else if((zf != tstate)&&(zb == tstate)) z = minval;
  else z = midval;

  if(x >= maxval) positions[robot_position_arm] -= adder;
  else if(x <= minval) positions[robot_position_arm] += adder;
  else{}

  if(y <= minval) positions[robot_position_boom] += adder;
  else if(y >= maxval) positions[robot_position_boom] -= adder;
  else{}

  if(z <= minval) positions[robot_position_base] += adder;
  else if(z >= maxval) positions[robot_position_base] -= adder;
  else{}

  steppers.moveTo(positions); //포지션 입력
  steppers.runSpeedToPosition(); //각각 동작
}
