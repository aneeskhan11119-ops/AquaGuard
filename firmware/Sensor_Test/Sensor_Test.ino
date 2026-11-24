// AquaGuard sensor test - checks both JSN-SR04T sensors, nothing else.
// Upload, open Serial Monitor at 115200 baud, and read the lines.

const int TRIG1 = 26, ECHO1 = 27;   // Sensor 1 (source tank)
const int TRIG2 = 25, ECHO2 = 33;   // Sensor 2 (roof tank)

float readCm(int trig, int echo) {
  digitalWrite(trig, LOW);  delayMicroseconds(5);
  digitalWrite(trig, HIGH); delayMicroseconds(20);   // JSN-SR04T needs a longer pulse
  digitalWrite(trig, LOW);
  unsigned long t = pulseIn(echo, HIGH, 30000);      // wait max 30 ms
  if (t == 0) return -1;                             // no echo
  return t * 0.0343 / 2.0;
}

void setup() {
  Serial.begin(115200);
  pinMode(TRIG1, OUTPUT); pinMode(ECHO1, INPUT);
  pinMode(TRIG2, OUTPUT); pinMode(ECHO2, INPUT);
  delay(500);
  Serial.println("Sensor test started");
}

void loop() {
  float d1 = readCm(TRIG1, ECHO1);
  delay(80);                                         // let echoes die out
  float d2 = readCm(TRIG2, ECHO2);

  Serial.print("Sensor 1 (D26/D27): ");
  if (d1 < 0) Serial.print("NO ECHO"); else { Serial.print(d1, 1); Serial.print(" cm"); }
  Serial.print("   |   Sensor 2 (D25/D33): ");
  if (d2 < 0) Serial.println("NO ECHO"); else { Serial.print(d2, 1); Serial.println(" cm"); }

  delay(500);
}
