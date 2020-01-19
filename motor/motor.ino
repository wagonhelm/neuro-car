int leftF = 9;
int rightF = 10;
int moveF = 6;
int moveL = 7;
int moveR = 8;
int pullOver = 5;

void setup() {
  // declare the ledPin as an OUTPUT:
  pinMode(leftF, OUTPUT);
  pinMode(rightF, OUTPUT);  
  pinMode(moveF, INPUT);
  pinMode(moveL, INPUT);
  pinMode(moveR, INPUT);
  pinMode(pullOver, INPUT);
}

void loop() {
  // Get forward control pin value
  int fValue = digitalRead(moveF);
  if (fValue == HIGH) {
    digitalWrite(leftF, HIGH);
    digitalWrite(rightF, HIGH);
    delay(500);
    digitalWrite(leftF, LOW);
    digitalWrite(rightF, LOW);
    delay(500);
  }

  int rValue = digitalRead(moveR);
  if (rValue == HIGH) {
    digitalWrite(rightF, HIGH);
    delay(200);
    digitalWrite(rightF, LOW);
    delay(200);
  }

  int lValue = digitalRead(moveL);
  if (lValue == HIGH) {
    digitalWrite(leftF, HIGH);
    delay(200);
    digitalWrite(leftF, LOW);
    delay(200);
  }

  int poValue = digitalRead(pullOver);
  if (poValue == HIGH) {
    digitalWrite(leftF, HIGH);
    delay(200);
    digitalWrite(rightF, HIGH);
    delay(1000);
    digitalWrite(leftF, LOW);
    delay(300);
    digitalWrite(rightF, LOW);
    delay(500);
  }
}
