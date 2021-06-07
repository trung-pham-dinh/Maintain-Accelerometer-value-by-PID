#include<PID_v1.h>
#include<MPU9255.h>

#define DIRA1 4 // use for IN3
#define DIRA2 5 // use for IN4
#define ENA 9 // use to control speed

MPU9255 mpu;

// for PID
double kp = 5.5, ki = 0.1, kd = 0.15;
double setPoint = 0, input = 0, output = 0;
int outputPWM = 0;
PID pid(&input, &output, &setPoint, kp, ki, kd, DIRECT);

int dir = 0;
void setup() {
  Serial.begin(9600);
  
  pinMode(DIRA1, OUTPUT);
  pinMode(DIRA2, OUTPUT);
  pinMode(ENA, OUTPUT);

  if(mpu.init())
  {
  Serial.println("initialization failed");
  }
  else
  {
  Serial.println("initialization successful!");
  }

  setPoint = 0;
  input = 0;
  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(-255,255);
}

void loop() {
  // read input
  mpu.read_acc();//get data from the accelerometer (-16200, 16700)
  input = -map(mpu.ay, -16200, 16700, -90, 90);
  if(abs(input) < 10) {
    input = 0;
  }
  // PID calculate
  pid.Compute();

  // output
  if(output < 0) {
    outputPWM = map(output, -255, 0 , -255, -30);// 30 is minimum PWM that motor can rotate
    setMotor(1, outputPWM);
  }
  else {
    outputPWM = map(output, 0, 255 , 30, 255);
    setMotor(-1, outputPWM);
  }

  
  Serial.print("Current angle: ");
  Serial.print(input);
  Serial.print("    ");
  Serial.print("Output PWM: ");
  Serial.println(outputPWM);
}

void setMotor(int d, int s) {
  if(abs(s) < 36) { // acctually at 35, the motor stop
    analogWrite(ENA, 0);
  }
  else {
    analogWrite(ENA, abs(s));
  }
  if(d == 1) {
    digitalWrite(DIRA1, HIGH);
    digitalWrite(DIRA2, LOW);
    dir = 1;
  }
  else if (d == -1) {
    digitalWrite(DIRA1, LOW);
    digitalWrite(DIRA2, HIGH);
    dir = -1;
  }
  else {
    digitalWrite(DIRA1, LOW);
    digitalWrite(DIRA2, LOW);
    dir = 0;
  }
}
