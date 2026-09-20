/*
 * cnc구조 관련 링크
 * http://john-home.iptime.org:8085/xe/index.php?mid=board_jalN13&document_srl=1060
 * 아두이노 cnc쉴드 써서 제작 xyz라고 쓰여진데 있어서 하나 다른데만 안넣으면됨
 * z가 베이스
 * y가 링크1 암
 * x가 링크2 붐
 * 28byj모터 그리퍼 사용(쉴드내 z 11 y 10 x 9 spnen 12 을 각각 uln에 연결해서 씀)
 * a4988 드라이버 사용
 * 핀맵은 하단부에 적어둠
 * 동작에 앞서 led를 켜고 끄게 해둠
 * accelstepper, stepper 라이브러리 같이쓰기
 * 8번핀 enable gnd 동작이라서 스위치 연결하면 전원 항시 연결에 동작 활성화 비활성화 할수있음
 * a4a5에 조이스틱 연결
 * d10에 스위치 연결
 * 
 * 마이크로 스텝
 * 분주비 16:1
 * 감속기어 비율 5:1
 * 작은 기어 회전에 걸리는 풀 스텝 수 200
 * 큰기어와 작은 기어간 기어비 9:32 - 711
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
    acceleration //가속도
    currentPosition //현재 위치
    speed //현재 속도
    distanceToGo //현재에서 목표까지의 거리
    targetPosition //최근에 설정한 목표

*/








#include <AccelStepper.h>
#include <MultiStepper.h>

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

#define robot_position_base 0
#define robot_position_arm 1
#define robot_position_boom 2

  int minval = 350, maxval = 600, motor_speed_offset = 3000,
  swstate = 0, dirset_arm = 0, dirset_boom = 0,dirset_base = 0;
  long position_arm = 0,position_boom = 0,position_base = 0;

long positions[3];

void armturn();
void boomturn();
void baseturn();
void speedset();
void manualcontroller();
void manualcontroller2();

AccelStepper arm = AccelStepper(1, STEP0, DIR0);//링크2번
AccelStepper boom = AccelStepper(1, STEP01, DIR01);//링크1번
AccelStepper base = AccelStepper(1, STEP02, DIR02);//베이스회전

MultiStepper steppers;

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

  Serial.begin(57600);

  base.setMaxSpeed(10000);
  boom.setMaxSpeed(10000);
  arm.setMaxSpeed(10000);

  base.setCurrentPosition(0);
  boom.setCurrentPosition(0);
  arm.setCurrentPosition(0);

  steppers.addStepper(base);
  steppers.addStepper(boom);
  steppers.addStepper(arm);
  
  Serial.print("initiate robotarm");
}



void loop()
{
  modeselect();

  positions[0] = 0;//베이스 위치 설정
  positions[1] = 1000;//암 위치 설정
  positions[2] = 2000;//붐 위치 설정

  steppers.moveTo(positions); //포지션 입력
  steppers.runSpeedToPosition(); //각각 동작

//  boom.setMaxSpeed(10000);
//  arm.setMaxSpeed(10000);

  positions[1] = 2000;//암 위치 설정
  positions[2] = 4000;//붐 위치 설정

  steppers.moveTo(positions); //포지션 입력
  steppers.runSpeedToPosition(); //각각 동작

//  boom.setMaxSpeed(1000);
//  arm.setMaxSpeed(1000);

  positions[1] = 3000;//암 위치 설정
  positions[2] = 6000;//붐 위치 설정

  steppers.moveTo(positions); //포지션 입력
  steppers.runSpeedToPosition(); //각각 동작

  delay(1000);
  
  positions[0] = 0;//베이스 위치 설정
  positions[1] = 0;//암 위치 설정
  positions[2] = 0;//붐 위치 설정

  steppers.moveTo(positions); //포지션 입력
  steppers.runSpeedToPosition(); //각각 동작
  delay(1000);

}











void modeselect()
{
  if(digitalRead(sw)==0)
  {
    swstate+=1;
    delay(200);
  }

  if(swstate == 3)swstate = 0;
  else if(swstate == 0){}
  else if(swstate == 1)
  {
    Serial.print("initiate robotarm manual mode");
    delay(1000);
    for(;;)
    {
      manualcontroller2();
      if(digitalRead(sw)==0)
      {
        swstate+=1;
        Serial.print("MANUAL CONTROL OUT");
        delay(200);
        return;
      }
    }
  }
  else if(swstate == 2)
  {
    //포지션 입력 등등
  }
  else{}
}

void manualcontroller2()
{
  int x = analogRead(xval);
  int y = analogRead(yval);
  int adder = 10;
  
  if(x >= maxval)
  {
    positions[1] += adder;
  }

  else if(x <= minval)
  {
    positions[1] -= adder;
  }
  else{}
  
  if(y <= minval)
  {
    positions[2] += adder;
  }
  
  else if(y >= maxval)
  {
    positions[2] -= adder;
  }
  else{}

  steppers.moveTo(positions); //포지션 입력
  steppers.runSpeedToPosition(); //각각 동작
  
}

void manualcontroller()
{
  
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
