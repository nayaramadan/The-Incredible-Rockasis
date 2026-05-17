int xPinDir = 0;
int yPinDir = 4;
int SWPinDir = 3;

int xPinThrot = 2;
int yPinThrot = 1;
int SWPinThrot = 5;

int xValDir;
int yValDir;
int SWValDir;

int xValThrot;
int yValThrot;
int SWValThrot;

void setup() {
  Serial.begin(115200);
  pinMode(xPinDir, INPUT);
  pinMode(yPinDir, INPUT);
  pinMode(SWPinDir, INPUT_PULLUP);

  pinMode(xPinThrot, INPUT);
  pinMode(yPinThrot, INPUT);
  pinMode(SWPinThrot, INPUT_PULLUP);
}

void loop() {
  xValDir = analogRead(xPinDir);
  yValDir = analogRead(yPinDir);
  SWValDir = digitalRead(SWPinDir);

  xValThrot = analogRead(xPinThrot);
  yValThrot = analogRead(yPinThrot);
  SWValThrot = digitalRead(SWPinThrot);

  Serial.print("Direction --   X : ");
  Serial.print(xValDir);
  Serial.print("   Y : ");
  Serial.print(yValDir);
  Serial.print("   Button : ");
  Serial.println(SWValDir);


  Serial.print("Throttle --    X : ");
  Serial.print(xValThrot);
  Serial.print("   Y : ");
  Serial.print(yValThrot);
  Serial.print("   Button : ");
  Serial.println(SWValThrot);

  Serial.print("\n");
  delay(100);

}