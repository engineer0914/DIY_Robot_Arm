/*

패치노트
  
  13. 설명 모드 추가, on off 추가
  12. 관절별 포지션 주소 지정 오류 수정 - 1번 모터가 돌아야 하는데 2번 모터가 돈다? = position 배열 숫자 수정 할것
  11. 모드 변경 버튼 조이스틱 통합
  10. 역기구학 구현
  09. 정기구학 구현 및 중도 오차 수정
  08. 아두이노 메가로 변경 및 버튼방식으로 변경
  07. 모드 설정 및 조이스틱 무브먼트 개선 = 시작과 동시에 조이스틱 입력시 모드전환
  06. 각도 입력 전 초기 위치 설정
  05. 감속기어 부착 버전 각도 입력 및 동시 도착
  04. 각 모터별 각도 입력 및 동시 도착
  03. 조이스틱 무브먼트 속도 개선, 설정 가능
  02. 동시동작 동시 도착
  01. 조이스틱 무브먼트

*/

/*
 * cnc구조 관련 링크
 * http://john-home.iptime.org:8085/xe/index.php?mid=board_jalN13&document_srl=1060
 * 아두이노 cnc쉴드 써서 제작 xyz라고 쓰여진데 있어서 하나 다른데만 안넣으면됨
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
 * 큰기어와 작은 기어간 기어비 9:32 - 711
 * NEMA 17급 바이폴라 방식 마이크로스텝 구동
 */

/*

multistepper


//////%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
addStepper (AccelStepper &stepper)
//////%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Add a stepper to the set of managed steppers There is an upper limit of MULTISTEPPER_MAX_STEPPERS = 10 to the number of steppers that can be managed
    Parameters [in]  stepper Reference to a stepper to add to the managed list
    
    관리되는 스테퍼 세트에 스테퍼 추가 관리 가능한 스테퍼 수에 MISTEPER_MAX_STEPPER = 10개의 상한이 있습니다
    매개변수 [in] stepper 관리 목록에 추가할 stepper 참조
//////%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
moveTo (long absolute[])
//////%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Set the target positions of all managed steppers according to a coordinate array. New speeds will be
    computed for each stepper so they will all arrive at their respective targets at very close to
    the same time.
    Parameters [in]  absolute  An array of desired absolute stepper positions. absolute[0] will
    be used to set the absolute position of the first stepper added by addStepper() etc. The array
    must be at least as long as the number of steppers that have been added by addStepper, else results
    are undefined.
    
    좌표 배열에 따라 관리되는 모든 스텝의 목표 위치를 설정합니다. 각 스텝마다 새로운 속도가 계산되어 각 스텝이 거의 동시에
    각 목표에 도달합니다.
    매개변수 absolute [in] 원하는 절대 스테퍼 위치의 배열입니다. absolute [0]은 addStepper() 등에 의해 추가된
    첫 번째 스테퍼의 절대 위치를 설정하는 데 사용됩니다. 배열은 적어도 addStepper에 의해 추가된 스테퍼의 수만큼 길어야 하며,
    그 외의 결과는 정의되지 않습니다. 배열을 먼저 정하고 이후에 함수 호출하면 지정된 배열에따라 각자 위치가 각 스텝마다 지정
    이후 동작
//////%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
run ()
//////%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Calls runSpeed() on all the managed steppers that have not acheived their target position.
    Returns true if any stepper is still in the process of running to its target position.
    
    호출은 목표 위치를 달성하지 못한 모든 관리되는 스텝에서 Speed()를 실행합니다.
    반환 스텝퍼가 아직 목표 위치로 실행 중인 경우 true입니다.
//////%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
runSpeedToPosition ()
//////%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Runs all managed steppers until they acheived their target position. Blocks until all that position is acheived. If you dont want blocking consider using run() instead.
    
    목표 위치를 달성할 때까지 관리되는 모든 스텝을 실행합니다. 해당 위치가 모두 달성될 때까지 차단합니다. 차단을 원하지 않는다면 대신 run()을 사용하는 것을 고려하십시오.
//////%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


accel stepper


////////
//설정
////////

//////%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
setMaxSpeed(float speed)
//////%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Sets the maximum permitted speed. The run() function will accelerate up to the speed set by
    this function. Caution: the maximum speed achievable depends on your processor and clock speed.
    The default maxSpeed is 1.0 steps per second.
    Parameters [in]  speed The desired maximum speed in steps per second. Must be > 0. Caution: Speeds
    that exceed the maximum speed supported by the processor may Result in non-linear accelerations and
    decelerations.
    
    허용되는 최대 속도를 설정합니다. run() 기능은 이 기능에 의해 설정된 속도까지 가속됩니다. 주의: 달성 가능한 최대 속도는
    프로세서와 클럭 속도에 따라 다릅니다. 기본 maxSpeed는 초당 1.0단계입니다.
    매개변수 [in] speed 원하는 최대 속도(초당 단계). 0 이상이어야 합니다. 주의: 프로세서가 지원하는 최대 속도를 초과하는
    속도는 비선형 가속 및 감속을 초래할 수 있습니다.

//////%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
setAcceleration(float acceleration)
//////%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Sets the acceleration/deceleration rate.
    Parameters [in]  acceleration  The desired acceleration in steps per second per second. Must be > 0.0.
    This is an expensive call since it requires a square root to be calculated. Dont call more 
    of then than needed
    
    가속/감속 속도를 설정합니다.
    매개변수 [in] 가속도 초당 단계별 원하는 가속도. > 0.0이어야 합니다. 계산하려면 제곱근이 필요하기 때문에 값비싼
    호출입니다. 필요한 것보다 더 많이 호출하지 마십시오

//////%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
setSpeed(float speed)
//////%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Sets the desired constant speed for use with runSpeed().
    Parameters [in]  speed The desired constant speed in steps per second. Positive is clockwise.
    Speeds of more than 1000 steps per second are unreliable. Very slow speeds may be set
    (eg 0.00027777 for once per hour, approximately. Speed accuracy depends on the Arduino crystal.
    Jitter depends on how frequently you call the runSpeed() function. The speed will be limited by
    the current value of setMaxSpeed()

    runSpeed()와 함께 사용할 원하는 일정 속도를 설정합니다.
    매개변수 speed 초당 원하는 일정한 속도(단계)입니다. 양수는 시계 방향입니다. 초당 1000보 이상의 속도는
    신뢰할 수 없습니다. 매우 느린 속도가 설정될 수 있습니다(예: 대략 한 시간에 한 번 0.00027777). 속도 정확도는
    아두이노 크리스탈에 달려 있습니다. 지터는 runSpeed() 함수를 얼마나 자주 부르는지에 따라 달라집니다.
    속도는 setMaxSpeed()의 현재 값에 의해 제한됩니다

//////%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
setCurrentPosition(long position)
//////%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Resets the current position of the motor, so that wherever the motor happens to be right now is
    considered to be the new 0 position. Useful for setting a zero position on a stepper after an
    initial hardware positioning move. Has the side effect of setting the current motor speed to 0.
    Parameters [in]  position  The position in steps of wherever the motor happens to be right now.
    
    모터의 현재 위치를 재설정하여 현재 모터가 있는 곳을 새로운 0 위치로 간주합니다. 초기 하드웨어 위치 지정 이동 후
    스텝에서 0 위치를 설정하는 데 유용합니다. 현재 모터 속도를 0으로 설정하는 부작용이 있습니다.    
    매개변수 [in] 위치 모터가 지금 있는 곳의 단계별 위치를 지정합니다.

////////
//동작
////////

//////%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
run
//////%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Poll the motor and step it if a step is due, implementing accelerations and decelerations to achieve
    the target position. You must call this as frequently as possible, but at least once per minimum
    step time interval, preferably in your main loop. Note that each call to run() will make at most one
    step, and then only when a step is due, based on the current speed and the time since the last step.
    Returns
    true if the motor is still running to the target position.
    
    모터를 폴링하고 스텝이 필요한 경우 스텝을 밟아서 목표 위치에 도달하기 위해 가속 및 감속을 구현합니다. 가능한 한 자주
    호출해야 하지만 최소 스텝 시간 간격마다 적어도 한 번씩은 메인 루프에서 호출해야 합니다. 각 실행 호출은 현재 속도와
    마지막 스텝 이후의 시간을 기반으로 최대 한 스텝을 수행한 다음 스텝이 필요한 경우에만 수행됩니다.
    반환 모터가 여전히 목표 위치로 작동 중인 경우 true입니다.

    
//////%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
runSpeed
//////%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Poll the motor and step it if a step is due, implementing a constant speed as set by the most
    recent call to setSpeed(). You must call this as frequently as possible, but at least once per
    step interval,
    Returns true if the motor was stepped.
    
    모터를 폴링하고 스텝이 필요한 경우 스텝을 밟아서 가장 최근에 set Speed()를 호출하여 설정한 대로 일정한 속도를
    구현합니다. 가능한 한 자주 호출해야 하지만 스텝 간격마다 적어도 한 번은 호출해야 합니다,
    반환 모터가 밟혔으면 참입니다.

//////%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
move(long relative)
//////%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Set the target position relative to the current position.
    Parameters [in]  relative  The desired position relative to the current position.
    Negative is anticlockwise from the current position.
    
    목표 위치를 현재 위치에 상대적으로 설정합니다.
    매개변수 relative 현재 위치 대비 원하는 위치. 음수는 현재 위치에서 시계 반대 방향입니다.

//////%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
moveTo(long absolute)
//////%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Set the target position. The run() function will try to move the motor  (at most one step per 
    call) from the current position to the target position set by the most recent call to this 
    function. Caution: moveTo() also recalculates the speed for the next step. If you are trying to
    use constant speed movements, you should call setSpeed() after calling moveTo().
    Parameters [in]  absolute  The desired absolute position. Negative is anticlockwise from the 0 position.
    
    목표 위치를 설정합니다. run() 함수는 모터를 현재 위치에서 이 함수에 대한 가장 최근 호출로 설정된 목표 위치로
    이동하려고 시도합니다. 주의: moveTo()는 다음 단계의 속도도 다시 계산합니다. 정속 이동을 사용하려는 경우
    moveTo()를 호출한 후 setSpeed()를 호출해야 합니다.
    매개변수 [in] absolute 원하는 절대 위치. 0 위치에서 negative는 시계 반대 방향입니다.

//////%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
runToPosition
//////%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Moves the motor (with acceleration/deceleration) to the target position and blocks until it is
    at position. Dont use this in event loops, since it blocks.
    
    모터(가속/감속 시)를 목표 위치로 이동시키고 위치에 도달할 때까지 차단합니다. 이벤트 루프에서는 차단

//////%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
runToNewPosition(long position)
//////%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Moves the motor (with acceleration/deceleration) to the new target position and blocks until
    it is at position. Dont use this in event loops, since it blocks.
    Parameters [in]  position  The new target position.
    
    모터(가속/감속 시)를 새로운 목표 위치로 이동시키고 위치에 도달할 때까지 차단합니다. 이벤트 루프에서는 차단
    매개변수 [in] 위치 새 대상 위치.

//////%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
stop
//////%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Sets a new target position that causes the stepper to stop as quickly as possible, using the
    current speed and acceleration parameters.
    
    현재 속도 및 가속도 매개변수를 사용하여 스텝퍼가 가능한 한 빨리 정지하도록 하는 새로운 목표 위치를 설정합니다.

////////
//값반환
////////

//////%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
maxSpeed
//////%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Returns the maximum speed configured for this stepper that was previously set by setMaxSpeed();
    Returns The currently configured maximum speed
    
    setMaxSpeed()에 의해 이전에 설정된 이 스텝퍼에 대해 구성된 최대 속도를 반환합니다;    
    반환 현재 구성된 최대 속도

    
//////%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
acceleration
//////%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Returns the acceleration/deceleration rate configured for this stepper that was previously set
    by setAcceleration();
    Returns The currently configured acceleration/deceleration
    
    setAcceleration()에 의해 이전에 설정된 이 스텝퍼에 대해 구성된 가속/감속 속도를 반환합니다;    
    반환 현재 구성된 가속/감속


//////%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
currentPosition
//////%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    The current motor position.
    Returns the current motor position in steps. Positive is clockwise from the 0 position.
    
    현재 모터 위치.
    반환 현재 모터 위치(단계). 포지티브는 0 위치에서 시계 방향입니다.

    
//////%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
speed
//////%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    The most recently set speed.
    Returns the most recent speed in steps per second

    가장 최근에 설정된 속도입니다.
    반환 가장 최근의 초속

    
//////%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
distanceToGo
//////%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    The distance from the current position to the target position.
    Returns the distance from the current position to the target position in steps. Positive is clockwise
    from the current position.
    
    현재 위치에서 목표 위치까지의 거리입니다.    
    반환 현재 위치에서 목표 위치까지의 거리(단계). 양은 현재 위치에서 시계 방향입니다.

    
//////%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
targetPosition
//////%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    The most recently set target position.
    Returns the target position in steps. Positive is clockwise from the 0 position.
    
    가장 최근에 설정한 목표 위치입니다.
    반환 단계별 목표 위치. Positive는 0 위치에서 시계 방향입니다.

*/

/*

multistepper

    addStepper (AccelStepper &stepper)//모터 추가
    moveTo (long absolute[])//따로 배열 불러서 위치 지정 이후 이 함수 부르면 각 스텝별 위치 지정
    run()//동작
    runSpeedToPosition ()//고정된 속도로 동작

accel stepper

    설정
    setMaxSpeed(float speed) //최대 제한 속도 설정
    setAcceleration(float acceleration) //가속도 설정
    setSpeed(float speed) //속도 설정
    setCurrentPosition(long position) //현재 위치 설정
    
    동작
    run //그냥 돌림
    runSpeed //고정 속도 돌림
    move(long relative)//얼만큼
    moveTo(long absolute)//어디로
    runToPosition //현재위치로 이동
    runToNewPosition(long position) //지정된 새 위치로 이동
    stop //정지
    
    값반환
    maxSpeed //최대 제한 속도
    acceleration //가속도 - 근데 사용 안됨
    currentPosition //현재 위치
    speed //현재 속도
    distanceToGo //현재에서 목표까지의 거리
    targetPosition //최근에 설정한 목표

*/

#include <AccelStepper.h>
#include <MultiStepper.h>

#define ON 1
#define OFF 0

#define gval A3
#define zval A0
#define xval A1
#define yval A2

#define xbuf 22
#define xbub 23
#define ybuf 24
#define ybub 25
#define zbuf 26
#define zbub 27
#define gbuf 28
#define gbub 29

#define DIR0 5
#define STEP0 2
#define DIR01 6
#define STEP01 3
#define DIR02 7
#define STEP02 4

#define robot_position_base 0
#define robot_position_boom 1
#define robot_position_arm  2

#define smallgearstep    200    //one circle for step
#define microstep        16     //microstep
#define normgear         3.56   //32/9 = 3.5556
#define planetarygear    1      //5:1

#define accelerator      OFF      //acclereation disable 0 enable 1
#define handcontroller   OFF      //0 joystick 1 button
#define gripper          OFF      //gripper on 1 off 0
#define description      OFF      //description 0 off 1 on

AccelStepper base = AccelStepper(1, STEP02, DIR02);//BASE
AccelStepper boom = AccelStepper(1, STEP0, DIR0);//BOOM
AccelStepper arm = AccelStepper(1, STEP01, DIR01);//ARM

MultiStepper steppers;

long positions[3];
int minval = 300, maxval = 700, midval = 500;//조이스틱 ADC 값 각각 최소 최대 중앙 값
float speedmotor = 1200.0;//제한 최대 속도
double armangle_current, boomangle_current, baseangle_current;//현재 각도 전역 변수
double out_boomangle, out_armangle;//역기구학 출력 각도 전역 변수
double posx = 12.0, posy = 12.0, posz=0.0, y_offset=12;//xy 평면 좌표, z 회전 각도, 로봇 붐 관절 중앙 지면 사이 길이
double boom_L1 = 12.0;//boom의 길이
double arm_L3 = 12.0;//arm의 길이
double endeffectoroffset_wd = 3.0;//arm의 끝과 엔드이펙터 사이의 거리

void defaultspeedset();//기본 속도 설정
void modeselect();//매뉴얼 컨트롤러 조이스틱, 버튼 인식시 수동 동작
void manualcontroller2();//매뉴얼 버튼 컨트롤러 모드
void manualcontroller3();//매뉴얼 조이스틱 컨트롤러 모드
void rotationmovementinsert(long base,long boom,long arm);//스텝 회전수 입력 동작
void anglemovementinset(double angle_base, double angle_boom, double angle_arm);//각도 입력 동작
void currentpositionshower();//현재 위치 각도 산출 함수
void Foward_Kinematics_Calculator(double angle_base, double angle_boom, double angle_arm);//정기구학 계산 입력:각도, 출력:좌표
void Foward_Kinematics_Action(double input_base,double input_boom,double input_arm); //정기구학 출력
void Inverse_Kinematics_Calculator(double P_x, double P_y);//역기구학 위치 계산 입력:좌표, 출력:각도
void Inverse_Kinematics_Action(double R_z,double Pos_x, double Pos_y);//역기구학 출력

void setup()
{
  //시리얼 모듈 활성화
  Serial.begin(57600);
  
  //포트 세팅
  pinMode(DIR0,OUTPUT);
  pinMode(STEP0,OUTPUT);
  pinMode(DIR01,OUTPUT);
  pinMode(STEP01,OUTPUT);
  pinMode(DIR02,OUTPUT);
  pinMode(STEP02,OUTPUT);
  
  if(handcontroller == OFF)//joy stick
  {
    pinMode(xval,INPUT);
    pinMode(yval,INPUT);
    if(gripper==1) pinMode(zval,INPUT);
    if(gripper==1) pinMode(gval,INPUT);
  }
  else if(handcontroller == ON)//button
  {
    pinMode(xbuf,INPUT);
    pinMode(xbub,INPUT);
    pinMode(ybuf,INPUT);
    pinMode(ybub,INPUT);
    if(gripper==1) pinMode(zbuf,INPUT);
    if(gripper==1) pinMode(zbub,INPUT);
    if(gripper==1) pinMode(gbuf,INPUT);
    if(gripper==1) pinMode(gbub,INPUT);
  }
  else{}

  Serial.print("\n initiate robotarm\n\n");

  //제한 최대 속도
  speedmotor = 1200.0;
  defaultspeedset();

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
  modeselect();// select manual controller it will move when you push it while arduino initiate first time

//  delay(10000);

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
    delay(3000);
  }
  
  angle_arm = radians(angle_arm);
  angle_boom = radians(angle_boom);

  double x_boom = boom_L1*sin(angle_boom);
  double y_boom = boom_L1*cos(angle_boom)+y_offset;

  double x_arm = x_boom+(arm_L3*cos(angle_arm));
  double y_arm = y_boom-(arm_L3*sin(angle_arm));

  double x_end = x_arm+endeffectoroffset_wd;
  double y_end = y_arm;

  if(description == ON)
  {
    Serial.println();
    Serial.print("output x boom");
    Serial.println(x_boom);
    Serial.print("output y boom");
    Serial.println(y_boom);
    delay(3000);
    
    Serial.println();
    Serial.print("output x arm");
    Serial.println(x_arm);
    Serial.print("output y arm");
    Serial.println(y_arm);
    delay(3000);
    
    Serial.println();
    Serial.print("output x end:");
    Serial.println(x_end);
    Serial.print("output y end:");
    Serial.println(y_end);
    delay(3000);
  }
  
}

void Inverse_Kinematics_Action(double R_z,double Pos_x, double Pos_y)
{
  if(description == ON)
  {
    Serial.println();
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
    delay(2000);
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
  if(accelerator == 1){}
  else rotationmovementinsert((long)instantba, (long)instantbo, (long)instantar);
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

void defaultspeedset()
{
  base.setMaxSpeed(speedmotor);
  if(planetarygear==5) speedmotor = 3000.0;
  boom.setMaxSpeed(speedmotor);
  arm.setMaxSpeed(speedmotor);
}

void modeselect()
{
  if(handcontroller == OFF)//joystick
  {
    int x = analogRead(xval);
    int y = analogRead(yval);
    if((x <= maxval)&&(x >= minval))return;
    if((y <= maxval)&&(y >= minval))return;
    Serial.print("\n initiate robotArm manual mode \n press reset to exit manual mode and reset robotArm");
    for(;;) manualcontroller2();
  }
  else if(handcontroller == ON)//button control
  {
    int x = digitalRead(xval);
    if(x != 0) return;
    Serial.print("\n initiate robotArm manual mode \n press reset to exit manual mode and reset robotArm");
    for(;;) manualcontroller3();
  }
  else{}
}

void manualcontroller2()//joy stick
{
  int adder = 10;
  int x = analogRead(xval);
  int y = analogRead(yval);
  int z = midval;
  //  int z = analogRead(zval);
  int g=midval;
  //  int g = analogRead(gval);

  if(x >= maxval) positions[robot_position_arm] -= adder;
  else if(x <= minval) positions[robot_position_arm] += adder;
  else{}

  if(y <= minval) positions[robot_position_boom] += adder;
  else if(y >= maxval) positions[robot_position_boom] -= adder;
  else{}

  if(z <= minval) positions[robot_position_base] += adder;
  else if(z >= maxval) positions[robot_position_base] -= adder;
  else{}

  if(gripper == ON)
  {
    if(g <= minval) positions[robot_position_base] += adder;
    else if(g >= maxval) positions[robot_position_base] -= adder;
    else{}
  }

  steppers.moveTo(positions); //포지션 입력
  steppers.runSpeedToPosition(); //각각 동작
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

  if(gripper == ON)
  {
    if((zf == tstate)&&(zb != tstate)) z = maxval;
    else if((zf != tstate)&&(zb == tstate)) z = minval;
    else z = midval;
    
    if((gf == tstate)&&(gb != tstate)) g = maxval;
    else if((gf != tstate)&&(gb == tstate)) g = minval;
    else g = midval;
  }

  if(x >= maxval) positions[robot_position_arm] -= adder;
  else if(x <= minval) positions[robot_position_arm] += adder;
  else{}

  if(y <= minval) positions[robot_position_boom] += adder;
  else if(y >= maxval) positions[robot_position_boom] -= adder;
  else{}

  if(gripper == ON)
  {
    if(z <= minval) positions[robot_position_base] += adder;
    else if(z >= maxval) positions[robot_position_base] -= adder;
    else{}
    
    if(g <= minval) positions[robot_position_base] += adder;
    else if(g >= maxval) positions[robot_position_base] -= adder;
    else{}
  }

  steppers.moveTo(positions); //포지션 입력
  steppers.runSpeedToPosition(); //각각 동작
}
