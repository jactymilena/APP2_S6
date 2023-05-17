
void setup() {
  // put your setup code here, to run once:
  // Serial.begin(9600);
  Serial.begin(115200);
  Serial.println("Hello World!");


}

void loop() {
  // put your main code here, to run repeatedly:
  int value = analogRead(34);
  //value = map(value, 0, 800, 0, 10);
  Serial.println(value);

  //bar.setLevel(value);
  delay(100);
}
