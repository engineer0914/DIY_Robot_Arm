/*
 * Incheon National University
 * Project:    PARALLEL ROBOT ARM CONTROL
 * Laboratory: Robotics & Energy Convergence Lab
 * Date:       2024 FALL
 * Professor:  Wooyong Kim
 * Assistant:  Dongsik Park
 * 
 * mEmO
 * cnc구조 관련 링크
 * http://john-home.iptime.org:8085/xe/index.php?mid=board_jalN13&document_srl=1060
 * 아두이노 cnc쉴드 써서 제작 xyz라고 쓰여진데 있어서 하나 다른데(붉은 부분)만 안넣으면됨
 * z가 베이스
 * y가 링크1 암
 * x가 링크2 붐
 * a4988 드라이버 사용
 * 핀맵은 하단부에 적어둠
 * accelstepper, multistepper 라이브러리 같이쓰기
 * d8 enable 기능 gnd 동작이라서 스위치 연결하면 전원 항시 연결에 동작 활성화 비활성화 할수있음
 * a0123에 조이스틱 연결
 * d22~29에 스위치 연결
 * 마이크로 스텝 활성화시 점퍼캡 사용할 것
 * 
 * 마이크로 스텝
 * 분주비 16:1
 * 감속기어 비율 5:1
 * 작은 기어 회전에 걸리는 풀 스텝 수 200
 * 큰기어와 작은 기어간 기어비 9:32 - 3.555 * 200 = 711
 * NEMA 17급 / 바이폴라 방식 / 마이크로스텝 구동
 * 파워 서플라이 12V 5A
 * StandAlone시 레귤레이터 부착 요구
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

#define description         OFF     //description 0 off 1 on

AccelStepper base = AccelStepper(1, STEP02, DIR02);//BASE
AccelStepper boom = AccelStepper(1, STEP0, DIR0);//BOOM
AccelStepper arm = AccelStepper(1,STEP01, DIR01);//ARM

MultiStepper steppers;

int N1 = 0, N2 = 0;//프로토콜 입력 전역 변수
String serString = "";//프로토콜 문자열
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
void serialProc();//시리얼 프로토콜 모드
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
  Serial.begin(57600);

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
        Inverse_Kinematics_Action(0,manual_x, manual_y);
      }
    }
  }
  else if(modenum == 3)
  {
    Serial.print("Protocol mode initiate");
    for(;;)
    {
      serialProc();
    }
//    perfomance_1();
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
  Inverse_Kinematics_Action(90,15,0);
  
  Inverse_Kinematics_Action(90,15,24);

  Inverse_Kinematics_Action(-90,15,24);

  Inverse_Kinematics_Action(-90,15,0);

  Inverse_Kinematics_Action(0,15,24);

  delay(5000);
}

void perfomance_2()//한붓 그리기
{
  int j = 1;

/*
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
  // currentpositionshower();
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
    delay(2000);
  }
  Inverse_Kinematics_Calculator(Pos_x,Pos_y);
  anglemovementinset(R_z,out_boomangle,out_armangle);
}

void Inverse_Kinematics_Calculator(double P_x, double P_y)
{
  //역기구학 ppt에 있는 그림을 이용해서 보면 편함
  //double boom_L1 = 12.0;//boom의 길이
  //double arm_L3 = 12.0;//arm의 길이
  //double endeffectoroffset_wd = 3.0;//arm의 끝과 엔드이펙터 사이의 거리
  double boomangle_th1, armangle_th2, th21;
  P_y -= y_offset;//오프셋 제거
  double D = sqrt(pow((P_x) - endeffectoroffset_wd, 2) + pow(P_y, 2));//엔드이펙터 관절의 3차원 공간 위치
//  double posD = sqrt(pow(P_x, 2) + pow(P_y, 2));//엔드이펙터의 3차원 공간 위치
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
    Serial.println();
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

void serialProc() {
  while (Serial.available() > 0) {
    char ch = Serial.read();
    serString += ch;

    if (ch == '\n') {
      if (serString.length() > 0) {
        // SF 프로토콜 처리
        if (serString.startsWith("SF,") && serString.endsWith(",E\n")) {
          String dataString = serString.substring(3, serString.length() - 3); // 'SF,'와 ',E' 제거
          
          int commaIndex = dataString.indexOf(",");
          if (commaIndex != -1) {
            int N1 = dataString.substring(0, commaIndex).toInt();
            int N2 = dataString.substring(commaIndex + 1).toInt();

            // Forward Kinematics 실행
            Foward_Kinematics_Action(0, (double)N2, (double)N1);

            // 결과 전송
            Serial.print("SD,");
            Serial.print(N1);
            Serial.print(",");
            Serial.print(N2);
            Serial.print(",E\n");
          } else {
            Serial.println("Error: Invalid format. N1,N2 values missing.");
          }

        // SI 프로토콜 처리
        } else if (serString.startsWith("SI,") && serString.endsWith(",E\n")) {
          String dataString = serString.substring(3, serString.length() - 3); // 'SI,'와 ',E' 제거
          
          int commaIndex = dataString.indexOf(",");
          if (commaIndex != -1) {
            int B = dataString.substring(0, commaIndex).toInt();
            int A = dataString.substring(commaIndex + 1).toInt();

            // Inverse Kinematics 실행
            Inverse_Kinematics_Action(0, (double)B, (double)A);

            // out_boomangle과 out_armangle 값 전송
            Serial.print("SD,");
            Serial.print((int)out_boomangle);
            Serial.print(",");
            Serial.print((int)out_armangle);
            Serial.print(",E\n");
          } else {
            Serial.println("Error: Invalid format. B,A values missing.");
          }
        } else {
          Serial.println("Error: Invalid command format. Expected 'SF,B,A,E' or 'SI,B,A,E'.");
        }
      }
      serString = ""; // 명령어 문자열 초기화
    }
  }
}
