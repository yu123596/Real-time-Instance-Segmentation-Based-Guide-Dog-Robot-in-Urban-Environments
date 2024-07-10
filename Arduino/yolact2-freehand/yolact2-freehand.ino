String ball_axis_in = "";
String word0 = ""; //ID
String word1 = ""; //cx
String word2 = ""; //cy
String getValue(String data, char separator, int index);
boolean a = true;
int leftmotor = 4;
int leftmotorspeed = 5;
int rightmotor = 7;
int rightmotorspeed = 6;
int prex, oldprex = 60;
int x = 0;
int i = 0;
int speedvalue1 = 254;
int speedvalue2 = 175;
int speedvalue3 = 150;
int speedvalue4 = 125;
int speedvalue5 = 100;
int speedvalue6 = 75;
int speedvalue7 = 50;
int speedvalue0 = 0;

const int Walk_BUTTON = 22;  // R按鍵的接腳
const int R_BUTTON = 23;  // R按鍵的接腳
const int O_BUTTON = 24;  // R按鍵的接腳
const int OM_BUTTON = 25;  // R按鍵的接腳
const int Q_BUTTON = 26;  // R按鍵的接腳
const int H_BUTTON = 27;  // R按鍵的接腳
const int C_BUTTON = 28;  // R按鍵的接腳
const int D_BUTTON = 29;  // R按鍵的接腳
const int A_BUTTON = 30;  // R按鍵的接腳
const int B_BUTTON = 31;  // R按鍵的接腳
boolean Walk_buttonUp = true;
boolean R_buttonUp = true;
boolean O_buttonUp = true;
boolean OM_buttonUp = true;
boolean Q_buttonUp = true;
boolean H_buttonUp = true;
boolean C_buttonUp = true;
boolean D_buttonUp = true;
boolean A_buttonUp = true;
boolean B_buttonUp = true;

int Walk_buttonState  = false;
int Walk_buttonJump  = false;
int test_Walk_buttonJump  = false;
int R_buttonState = false;   // 按鈕的狀態
int O_buttonState = false;
int OM_buttonState = false;
int Q_buttonState = false;
int H_buttonState = false;
int C_buttonState = false;
int D_buttonState = false;   // 按鈕的狀態
int A_buttonState = false;
int B_buttonState = false;
int buzzer=3;                   //設定蜂鳴器接腳為第3孔
int duration = 100;
int aSo = 392;
int bDo = 523;
int bRe = 587;
int bMi = 659;
int bFa = 698;
int bSo = 784;
int bLa = 880;
int bSi = 988;
int bDDo = 1047;

int track(int x, int i);
void run_servo(int Lweel, int Rweel, int t);
void setup() {
  Serial.begin(9600);
  pinMode(leftmotor, OUTPUT);
  pinMode(rightmotor, OUTPUT);
  digitalWrite(leftmotor, HIGH);
  digitalWrite(rightmotor, HIGH);
  pinMode(R_BUTTON, INPUT_PULLUP);
  pinMode(O_BUTTON, INPUT_PULLUP);
  pinMode(OM_BUTTON, INPUT_PULLUP);
  pinMode(Q_BUTTON, INPUT_PULLUP);
  pinMode(Walk_BUTTON, INPUT_PULLUP);
  pinMode(H_BUTTON, INPUT_PULLUP);
  pinMode(C_BUTTON, INPUT_PULLUP);
  pinMode(D_BUTTON, INPUT_PULLUP);
  pinMode(A_BUTTON, INPUT_PULLUP);
  pinMode(B_BUTTON, INPUT_PULLUP);
  pinMode(buzzer,OUTPUT); 
}
void loop() {
  Walk_buttonState = digitalRead(Walk_BUTTON);  //讀取按鍵的狀態
  R_buttonState = digitalRead(R_BUTTON);  //讀取按鍵的狀態
  O_buttonState = digitalRead(O_BUTTON);  //讀取按鍵的狀態
  OM_buttonState = digitalRead(OM_BUTTON);  //讀取按鍵的狀態
  Q_buttonState = digitalRead(Q_BUTTON);  //讀取按鍵的狀態
  H_buttonState = digitalRead(H_BUTTON);  //讀取按鍵的狀態
  C_buttonState = digitalRead(C_BUTTON);  //讀取按鍵的狀態
  D_buttonState = digitalRead(D_BUTTON);  //讀取按鍵的狀態
  A_buttonState = digitalRead(A_BUTTON);  //讀取按鍵的狀態
  B_buttonState = digitalRead(B_BUTTON);  //讀取按鍵的狀態
  delay(50);
  if (Walk_buttonState == HIGH && Walk_buttonUp == true && Walk_buttonJump  == false) {        //如果Walk按鍵按下且按鈕狀態為true
    Walk_buttonUp = false;
  }
  if (Walk_buttonState == HIGH && Walk_buttonUp == false && Walk_buttonJump  == false ) {
    if (Serial.available()) {
      a = true;
      while (a) {
        char c = Serial.read();
        if (c == '\n') {
          a = false ;
        }
        else {
          ball_axis_in += c;
          delay(5);
        }
      }
      word0 = getValue(ball_axis_in, '_', 0);
      word1 = getValue(ball_axis_in, '_', 1);
      word2 = getValue(ball_axis_in, '_', 2);
      word2 = "0";
      if (word0 == "3") { // rgbid.elf track rgb line
        prex = word1.toInt();
        if (abs(prex - oldprex) < 1000)
        {
          x = prex / 10;
          oldprex = prex;
          i = track(x, i);
        }
        else
        {
          i = track(x, i);
        }
        // }
      }

      ball_axis_in = "";
    }
   //delay(50);
  }
  if (Walk_buttonState == HIGH && Walk_buttonUp == false && Walk_buttonJump  == true) {
    word0 = "3";
    word1 = 130;
    delay(5);
    if (word0 == "3") { // rgbid.elf track rgb line
        prex = word1.toInt();
        if (abs(prex - oldprex) < 1000)
        {
          x = prex / 10;
          oldprex = prex;
          i = track(x, i);
        }
        else
        {
          i = track(x, i);
        }
        // }
      }
      ball_axis_in = "";
    }
  if (Walk_buttonState == LOW && Walk_buttonUp == false && Walk_buttonJump  == false) {
    Walk_buttonUp = true;
    word0 = "3";
    word1 = 130;
    if (word0 == "3") { // rgbid.elf track rgb line
        prex = word1.toInt();
        if (abs(prex - oldprex) < 1000)
        {
          x = prex / 10;
          oldprex = prex;
          i = track(x, i);
        }
        else
        {
          i = track(x, i);
        }
        // }
      }

      ball_axis_in = "";
  }
  if (Walk_buttonState == LOW && Walk_buttonUp == false && Walk_buttonJump  == true) {
    Walk_buttonJump = false;
    Walk_buttonUp = true;
  }
  if (R_buttonState == LOW && R_buttonUp == true && O_buttonUp == true && O_buttonUp == true) {        //Road Detection mode按鍵
    Serial.println('R');
    R_buttonUp = false;
    delay(500);
  }
  if (O_buttonState == LOW && R_buttonUp == true && O_buttonUp == true && O_buttonUp == true) {        //Object Detection mode按鍵
    Serial.println('O');
    O_buttonUp = false;
    delay(500);
  }
  if (OM_buttonState == LOW && R_buttonUp == true && O_buttonUp == true && O_buttonUp == true) {        //Object Search mode按鍵
    Serial.println('M');
    OM_buttonUp = false;
    delay(500);
  }
  if (Q_buttonState == LOW && Q_buttonUp == true) {        //Quit mode按鍵
    Serial.println('Q');
    R_buttonUp = true;
    O_buttonUp = true;
    OM_buttonUp = true;
    Q_buttonUp = true;
    delay(500);
  }
  if (Q_buttonState == HIGH && Q_buttonUp == false) {    //Quit mode防彈跳
    Q_buttonUp = false;
    delay(500);
  }

  if (C_buttonState == LOW && C_buttonUp == true && OM_buttonUp == false) {        //如果按鍵按了
    Serial.println('C');
    C_buttonUp = false;
    delay(500);
  }
  if (C_buttonState == HIGH && C_buttonUp == false && OM_buttonUp == false) {         //如果按鍵按了
    Serial.println("end");
    C_buttonUp = true;
    delay(500);
  }

  if (H_buttonState == LOW && H_buttonUp == true && OM_buttonUp == false) {        //如果按鍵按了
    Serial.println('B');
    H_buttonUp = false;
    delay(500);
  }
  if (H_buttonState == HIGH && H_buttonUp == false && OM_buttonUp == false) {        //如果按鍵按了
    Serial.println("end");
    H_buttonUp = true;
    delay(500);
  }

  if (D_buttonState == LOW && D_buttonUp == true && OM_buttonUp == false) {        //如果按鍵按了
    Serial.println('P');
    D_buttonUp = false;
    delay(500);
  }
  if (D_buttonState == HIGH && D_buttonUp == false && OM_buttonUp == false) {        //如果按鍵按了
    Serial.println("end");
    D_buttonUp = true;
    delay(500);
  }

  if (A_buttonState == LOW && A_buttonUp == true && R_buttonUp == false) {         //如果按鍵按了
    Serial.println('A');
    A_buttonUp = false;
    delay(500);
  }
  if (A_buttonState == HIGH && A_buttonUp == false && R_buttonUp == false) {         //如果按鍵按了
    Serial.println("end");
    A_buttonUp = true;
    delay(500);
  }

  if (B_buttonState == LOW && B_buttonUp == true && R_buttonUp == false) {         //如果按鍵按了
    Serial.println('B');
    B_buttonUp = false;
    delay(500);
  }
  if (B_buttonState == HIGH && B_buttonUp == false && R_buttonUp == false) {         //如果按鍵按了
    Serial.println("end");
    B_buttonUp = true;
    delay(500);
  }
}

String getValue(String data, char separator, int index) {
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length() - 1;

  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }

  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

int track(int x, int i) {
  //Serial.println(String(x));
  switch (x) {
    case 20:
      //Serial.println("Finish");
      digitalWrite(leftmotor, HIGH);
      digitalWrite(rightmotor, HIGH);
      run_servo(speedvalue0, speedvalue0, 40);
      tone(3,1000,100);
      delay(200);
      tone(3,1000,100);
      delay(200);
      tone(3,1000,100);
      delay(200);
      tone(3,600,500);
      delay(200);
      tone(3,800,500);
      delay(200);
      tone(3,1200,750);
      i = 0;
      return i;
      break;
    case 19:
      Walk_buttonJump = true;
      //Serial.println("Crosswalk");
      digitalWrite(leftmotor, HIGH);
      digitalWrite(rightmotor, HIGH);
      run_servo(speedvalue0, speedvalue0, 40);
      tone(3,1000,750);
      delay(200);
      tone(3,1200,500);
      delay(200);
      tone(3,1000,250);
      delay(200);
      tone(3,1200,250);
      delay(500);
      tone(3,1000,750);
      delay(200);
      tone(3,1200,500);
      delay(200);
      tone(3,1000,250);
      delay(200);
      tone(3,1200,250);
      delay(200);
      i = 0;
      return i;
      break;
    case 18:
      Walk_buttonJump = true;
      //Serial.println("Up");
      digitalWrite(leftmotor, HIGH);
      digitalWrite(rightmotor, HIGH);
      run_servo(speedvalue0, speedvalue0, 40);
      delay(150);
      tone(3,aSo,duration);
      delay(150);
      tone(3,bRe,duration);
      delay(100);
      tone(3,bDDo,duration);
      delay(150);
      tone(3,bDDo,duration);
      i = 0;
      return i;
      break;
    case 17:
      Walk_buttonJump = true;
      //Serial.println("Down");
      digitalWrite(leftmotor, HIGH);
      digitalWrite(rightmotor, HIGH);
      run_servo(speedvalue0, speedvalue0, 40);
      delay(150);
      tone(3,aSo,duration);
      delay(150);
      tone(3,bRe,duration);
      delay(100);
      tone(3,aSo,duration);
      delay(150);
      tone(3,aSo,duration);
      i = 0;
      return i;
      break;
    case 16: //原地右旋轉
      digitalWrite(leftmotor, HIGH);
      digitalWrite(rightmotor, LOW);
      run_servo(speedvalue6, speedvalue3, 40);
      delay(150);
      tone(3,bRe,duration);
      delay(150);
      tone(3,bFa,duration);
      i = 0;
      return i;
      break;
    case 15: //原地左旋轉
      digitalWrite(leftmotor, LOW);
      digitalWrite(rightmotor, HIGH);
      run_servo(speedvalue3, speedvalue6, 40);
      delay(150);
      tone(3,bRe,duration);
      delay(150);
      tone(3,bDo,duration);
      i = 0;
      return i;
      break;
    case 14: //退後
      digitalWrite(leftmotor, LOW);
      digitalWrite(rightmotor, LOW);
      run_servo(speedvalue1, speedvalue1, 40);
      delay(150);
      tone(3,bDDo,duration);
      delay(150);
      tone(3,bDDo,duration);
      delay(100);
      tone(3,bDDo,duration);
      i = 0;
      return i;
      break;
    case 13: //停止
      digitalWrite(leftmotor, HIGH);
      digitalWrite(rightmotor, HIGH);
      run_servo(speedvalue0, speedvalue0, 40);
      delay(150);
      tone(3,bDDo,duration);
      delay(150);
      tone(3,bSi,duration);
      delay(100);
      tone(3,400,200);
      delay(100);
      i = 0;
      return i;
      break;
    case 12:
      digitalWrite(leftmotor, HIGH);
      digitalWrite(rightmotor, HIGH);
      run_servo(speedvalue1, speedvalue6, 40);
      tone(3,392,75);
      i = 0;
      return i;
      break;
    case 11:
      digitalWrite(leftmotor, HIGH);
      digitalWrite(rightmotor, HIGH);
      run_servo(speedvalue1, speedvalue5, 40);
      i = 0;
      return i;
      break;
    case 10:
      digitalWrite(leftmotor, HIGH);
      digitalWrite(rightmotor, HIGH);
      run_servo(speedvalue1, speedvalue4, 40);
      //tone(3,bSo,duration);
      i = 0;
      return i;
      break;
    case 9:
      digitalWrite(leftmotor, HIGH);
      digitalWrite(rightmotor, HIGH);
      run_servo(speedvalue1, speedvalue3, 40);
      i = 0;
      return i;
      break;
    case 8:
      digitalWrite(leftmotor, HIGH);
      digitalWrite(rightmotor, HIGH);
      run_servo(speedvalue1, speedvalue2, 40);
      //tone(3,bFa,duration);
      i = 0;
      return i;
      break;
    case 7:
      digitalWrite(leftmotor, HIGH);
      digitalWrite(rightmotor, HIGH);
      run_servo(speedvalue1, speedvalue1, 40);
      i = 0;
      return i;
      break;
    case 6:
      digitalWrite(leftmotor, HIGH);
      digitalWrite(rightmotor, HIGH);
      run_servo(speedvalue1, speedvalue1, 40);
      i = 0;
      return i;
      break;
    case 5:
      digitalWrite(leftmotor, HIGH);
      digitalWrite(rightmotor, HIGH);
      run_servo(speedvalue2, speedvalue1, 40);
      i = 0;
      return i;
      break;
    case 4:
      digitalWrite(leftmotor, HIGH);
      digitalWrite(rightmotor, HIGH);
      run_servo(speedvalue3, speedvalue1, 40);
      //tone(3,bRe,duration);
      i = 0;
      return i;
      break;
    case 3:
      digitalWrite(leftmotor, HIGH);
      digitalWrite(rightmotor, HIGH);
      run_servo(speedvalue4, speedvalue1, 40);
      i = 0;
      return i;
      break;
    case 2:
      digitalWrite(leftmotor, HIGH);
      digitalWrite(rightmotor, HIGH);
      run_servo(speedvalue5, speedvalue1, 40);
      //tone(3,bDo,duration);
      i = 0;
      return i;
      break;
    case 1:
      digitalWrite(leftmotor, HIGH);
      digitalWrite(rightmotor, HIGH);
      run_servo(speedvalue6, speedvalue1, 40);
      i = 0;
      return i;
      break;
    case 0:
      digitalWrite(leftmotor, HIGH);
      digitalWrite(rightmotor, HIGH);
      run_servo(speedvalue7, speedvalue1, 40);
      tone(3,330,75);
      i = 0;
      return i;
      break;
    default:
      digitalWrite(leftmotor, HIGH);
      digitalWrite(rightmotor, HIGH);
      run_servo(speedvalue1, speedvalue1, 20);
      i = 0;
      return i;
  }
}
void run_servo(int Lweel, int Rweel, int t) {

  analogWrite(leftmotorspeed, Lweel);
  analogWrite(rightmotorspeed, Rweel);
  unsigned long espT = millis();
  while ((millis() - espT) < t) {
    while (Serial.available()) {
      char cc = Serial.read();
    }
  }
}
