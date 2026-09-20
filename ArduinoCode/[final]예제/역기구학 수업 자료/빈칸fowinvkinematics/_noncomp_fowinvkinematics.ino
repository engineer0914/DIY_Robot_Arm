/*
 * Incheon National University
 * Project:    PARALLEL ROBOT ARM CONTROL
 * Laboratory: Robotics & Energy Convergence Lab
 * Date:       2024 FALL
 * Professor:  Wooyong Kim
 * Assistant:  Dongsik Park
 * 
 * 
 * motor 1 revolution with full step 200
 * micro step 16:1
 * reducer gear ratio 5:1
 * normal gear ratio 9:32 = 3.555
 * NEMA 17 / bipolar / micro step
 * power supply limit 12V 5A
 * StandAlone regulator needs
 *
 * Inverse Kinematics
 */

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

#define smallgearstep       200     //one circle for step
#define microstep           16      //microstep
#define normgear            3.56    //32/9 = 3.5556
#define planetarygear       5       //5:1

#define description         ON     //description 0 off 1 on

AccelStepper base = AccelStepper(1, STEP02, DIR02);//BASE
AccelStepper boom = AccelStepper(1, STEP0, DIR0);//BOOM
AccelStepper arm = AccelStepper(1,STEP01, DIR01);//ARM

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

void perfomance_1();
void perfomance_2();
void perfomance_3();
void perfomance_4();
void manualcontroller3();//매뉴얼 버튼 컨트롤러 모드
void rotationmovementinsert(long base,long boom,long arm);//스텝 회전수 입력 동작
void anglemovementinset(double angle_base, double angle_boom, double angle_arm);//각도 입력 동작
void currentpositionshower();//현재 위치 각도 산출 함수
void Foward_Kinematics_Calculator(double angle_base, double angle_boom, double angle_arm);//정기구학 계산 입력:각도, 출력:좌표
void Foward_Kinematics_Action(double input_base,double input_boom,double input_arm); //정기구학 출력
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

  Serial.print("\n initiate robotarm\n\n");

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

  for(;;)
  {
    Inverse_Kinematics_Action(0,15,0);
    delay(5000);
    Inverse_Kinematics_Action(0,15,24);
    delay(5000);
  }
  
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

  if(modenum == 1)//button
  {
    Serial.println();
    Serial.print("\n initiate robotArm manual mode \n press reset to exit manual mode and reset robotArm");
    Serial.println();
    Serial.print("button controller enable:");
    for(;;) manualcontroller3();
  }
  else if(modenum == 2)//serial controller
  {
    Serial.println();
    Serial.print("\n initiate robotArm manual mode \n press reset to exit manual mode and reset robotArm");
    Serial.println();
    Serial.print("serial controller enable:");
    for(;;)
    {
      if(Serial.available())
      {
        switch(Serial.read())
        {
          case'w': manual_y++; break;
          case's': manual_y--; break;
          case'a': manual_x--; break;
          case'd': manual_x++; break;
          case'q': break;//body turn
          case'e': break;//body turn
          case'f': break;//gripper
          case'g': break;//gripper
          default: break;//default nonmovement
        }
//        Inverse_Kinematics_Action(0,manual_x, manual_y);
      }
    }
  }
  else if(modenum == 3)
  {
    perfomance_1();
  }
  else if(modenum == 4)
  {
    perfomance_2();
    return;
  }
  else if(modenum == 5)
  {
    perfomance_3();
    return;
  }
  else if(modenum == 6)
  {
    perfomance_4();
    return;
  }
  else{}
  modenum = 0;
}

void perfomance_1()//픽앤 플레이스
{
  Inverse_Kinematics_Action(0,15,0);
  delay(10000);
}

void perfomance_2()//한붓 그리기
{
  //make u r move !
}

void perfomance_3()
{
  //make u r move !
}

void perfomance_4()
{
  //make u r move !
}

void Foward_Kinematics_Action(double input_base,double input_boom,double input_arm)
{
  Foward_Kinematics_Calculator(input_base,input_boom,input_arm);
  anglemovementinset(input_base,input_boom,input_arm);
  currentpositionshower();
}

void Foward_Kinematics_Calculator(double angle_base, double angle_boom, double angle_arm)
{
  //double boom_L1 = 12.0;//boom의 길이
  //double arm_L3 = 12.0;//arm의 길이
  //double endeffectoroffset_wd = 3.0;//arm의 끝과 엔드이펙터 사이의 거리
  
  if(description == ON)
  {
    Serial.println();
    Serial.print("input base angle:");
    Serial.println(angle_base);
    Serial.print("input boom angle:");
    Serial.println(angle_boom);
    Serial.print("input arm angle:");
    Serial.println(angle_arm);
    delay(1000);
  }

  angle_arm = radians(angle_arm);//입력 각도 도 -> 라디안 변환
  angle_boom = radians(angle_boom);//입력 각도 도 -> 라디안 변환

  double x_boom = boom_L1*sin(angle_boom);//붐의 끝부분 평면 좌표 x축 = sin(붐 각도)*붐 길이
  double y_boom = boom_L1*cos(angle_boom)+y_offset;//붐의 끝부분 평면 좌표 y축 = cos(붐 각도)*붐 길이 + 본체 높이 오프셋 길이

  double x_arm = x_boom+(arm_L3*cos(angle_arm));//암의 끝부분 평면 좌표 x축 = cos(암각도)*암 길이 + 붐 x좌표
  double y_arm = y_boom-(arm_L3*sin(angle_arm));//암의 끝부분 평면 좌표 y축 = sin(암각도)*암 길이*-1 + 붐 y좌표()세타 3가 기준이라 반대로 이동

  double x_end = x_arm+endeffectoroffset_wd;//최종 좌표 + 엔드이펙터 오프셋 거리
  double y_end = y_arm;//최종 좌표

  if(description == ON)//설명
  {
    Serial.println();
    Serial.print("output x boom");
    Serial.println(x_boom);
    Serial.print("output y boom");
    Serial.println(y_boom);
    
    Serial.println();
    Serial.print("output x arm");
    Serial.println(x_arm);
    Serial.print("output y arm");
    Serial.println(y_arm);
    
    Serial.println();
    Serial.print("output x end:");
    Serial.println(x_end);
    Serial.print("output y end:");
    Serial.println(y_end);
  }
}

void Inverse_Kinematics_Action(double R_z,double Pos_x, double Pos_y)
{
  if(description == ON)
  {
    Serial.print("input base angle:");
    Serial.println(R_z);
    Serial.print("input Pos_x:");
    Serial.println(Pos_x);
    Serial.print("input Pos_y:");
    Serial.println(Pos_y);
    delay(1000);
  }
  Inverse_Kinematics_Calculator(Pos_x,Pos_y);
  anglemovementinset(R_z,out_boomangle,out_armangle);
}

void Inverse_Kinematics_Calculator(double P_x, double P_y)
{
  //  double boomangle_th1, armangle_th2, th21, D, alpha, psi;
  //  y_offset, endeffectoroffset_wd, boom_L1, arm_L3
  //  P_x, P_y
  
  double boomangle_th1, armangle_th2, th21, D, alpha, psi;

  //1. 계산의 편의를 위해 엔드이펙터 관절부로 오프셋 차감 2



  //2. 피타고라스 정리를 이용하여 D 계산 1 


  //3. 제2 코사인 법칙을 통해 붐과 암 사이 각(th21) 구하기 1 


  //4. 제2 코사인 법칙을 통해 D와 붐 사이 각(alpha) 구하기 1 


  //5. 제2 코사인 법칙을 통해 지면과 붐 사이 각(beta = psi) 구하기 1


  //6. 계산된 3개의 라디안 도 변환 (deg = rad*180/PI) 3 




  //7. 붐 중앙축 y좌표 기준 엔드이펙터의 y좌표가 이상(alpha + psi)인지 이하(alpha - psi)별 수식 작성 = 붐 각도(boomangle_th1) 계산 4





  //8. 사잇각(th21) + 붐 각도(boomangle_th1) = 암 각도(armangle_th2) 계산 1


  //9. 모터에 맞는 기준축 간의 각도 값 변환((-1*boomangle_th1)+90, (-1*armangle_th2)+180 = out_boomangle, out_armangle) 2



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

  if((gf == tstate)&&(gb != tstate)) g = maxval;
  else if((gf != tstate)&&(gb == tstate)) g = minval;
  else g = midval;
  
  if(g <= minval) positions[robot_position_base] += adder;
  else if(g >= maxval) positions[robot_position_base] -= adder;
  else{}

  steppers.moveTo(positions); //포지션 입력
  steppers.runSpeedToPosition(); //각각 동작
}
