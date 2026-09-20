//모드 확인 코드

//핀번호 선언, 이렇게 선언하면 포트를 변경할때마다 쉽다.
#define xbuf    22
#define xbub    23
#define ybuf    24
#define ybub    25
#define zbuf    26
#define zbub    27
#define gbuf    28
#define gbub    29

void modecheckprint(int insert);//함수 선언 이렇게 모아두면 함수가 많을때 나중에 보기 편하다

void setup()//설정
{
  Serial.begin(9600);//출력을 위한 시리얼창 열기

  //각각 핀번호 입력 설정
  pinMode(xbuf,INPUT);
  pinMode(xbub,INPUT);
  pinMode(ybuf,INPUT);
  pinMode(ybub,INPUT);
  pinMode(zbuf,INPUT);
  pinMode(zbub,INPUT);
  pinMode(gbuf,INPUT);
  pinMode(gbub,INPUT);

  Serial.print("mode check code");
}

int modenum=0;//모드에 해당하는 전역변수

void loop()
{  
  for(;;)
  {
    //회로도 참고시 '풀다운' 스위치임에따라 버튼 입력시 1의 신호이기에 기준이 된다.
    int button_standard = 1;//버튼 입력 기준 신호 = 1(5v)
    //어떤 버튼이 입력되냐에 따라 달라지는 모드
    if(digitalRead(xbuf)!=button_standard) modenum = 1;//버튼 입력시 변수 변경
    if(digitalRead(xbub)!=button_standard) modenum = 2;
    if(digitalRead(ybuf)!=button_standard) modenum = 3;
    if(digitalRead(ybub)!=button_standard) modenum = 4;
    if(digitalRead(zbuf)!=button_standard) modenum = 5;
    if(digitalRead(zbub)!=button_standard) modenum = 6;
    if(modenum != 0) break;//만약 모드가 변경되면 반복문 탈출
  }
  
  if(modenum == 1)//n번 모드일때
  {
    modecheckprint(modenum);//모드 실행
  }
  else if(modenum == 2)
  {
    modecheckprint(modenum);//모드 실행
  }
  //지역 함수 선언 차이점: 긴 코드를 작성하는 것보다, 무엇을 하는 함수인지 작성하면 보기 깔끔하다.
  else if(modenum == 3)
  {
    Serial.print("mode:");
    Serial.println(modenum);//인자가 없어서 전역변수로 대체
    delay(500);
    modenum = 0;//변수 초기화에 따라 모드를 종료하고 버튼 입력 대기
  }
  else if(modenum == 4)
  {
    Serial.print("mode:");
    Serial.println(modenum);//인자가 없어서 전역변수로 대체
    delay(500);
    modenum = 0;//변수 초기화에 따라 모드를 종료하고 버튼 입력 대기
  }
  //이외 모드 번호에 따른 기능 else if 문으로 추가 가능
  else{}
}

void modecheckprint(int insert)//모드 확인을 위한 프린트 함수 요약
{
  Serial.print("mode:");
  Serial.println(insert);//인자로 받은 int형 insert값을 시리얼 모니터에 출력
  delay(500);
  modenum = 0;//변수 초기화에 따라 모드를 종료하고 버튼 입력 대기
}
