int speed = 0;
int lastOutput = 175;
int place = 0;

double kP = .1;
double kI = 0.0;
double kD = 0.0;

int target = 320;

void setup() {
  pinMode(11, OUTPUT);
  
  Serial.begin(9600);
}

void loop() {
  if (Serial.available()) {
    int readBit = Serial.read() - '0';  
    if (readBit == 'a' - '0') {
      speed = 0;
      place = 0;
    } else if (place == 0) {
      speed += 100 * readBit;
      place++;
    } else if (place == 1) {
      speed += 10 * readBit;
      place++;
    } else if (place == 2) {
       speed += readBit;
       
       int error = target - speed;
       
       double output = lastOutput + (kP * error);
       
       lastOutput = min(255, output);
       
       analogWrite(11, lastOutput);
       
       Serial.println(lastOutput);
    }
  }
}
