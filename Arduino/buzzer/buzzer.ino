#define Buzzer 9

void setup() {
 pinMode(Buzzer, OUTPUT);
}

void loop() {
  tone(Buzzer, 1000, 100);
  delay(200);
  tone(Buzzer, 1000, 100);
  delay(200);
  tone(Buzzer, 1000, 100);
  delay(200);
  tone(Buzzer, 1000, 100);
  delay(1000);
}
