// Pin definitions
#define MOTOR1_IN1 12
#define MOTOR1_IN2 14
#define MOTOR2_IN3 26
#define MOTOR2_IN4 27

// Motor control function
void controlMotor(int in1, int in2, int duration) {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  delay(duration);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
}

void setup() {
  Serial.begin(9600);

  // Set motor control pins as OUTPUT
  pinMode(MOTOR1_IN1, OUTPUT);
  pinMode(MOTOR1_IN2, OUTPUT);
  pinMode(MOTOR2_IN3, OUTPUT);
  pinMode(MOTOR2_IN4, OUTPUT);
}

void loop() {
  // Motor 1 on for 10 seconds
  Serial.println("Motor 1 ON for 10sec");
  controlMotor(MOTOR1_IN1, MOTOR1_IN2, 10000);

  // Both motors off for 20 seconds
  Serial.println("Both Motors OFF for 20sec");
  digitalWrite(MOTOR1_IN1, LOW);
  digitalWrite(MOTOR1_IN2, LOW);
  digitalWrite(MOTOR2_IN3, LOW);
  digitalWrite(MOTOR2_IN4, LOW);
  delay(20000);

  // Motor 2 on for 10 seconds
  Serial.println("Motor 2 ON for 10 sec ");
  controlMotor(MOTOR2_IN3, MOTOR2_IN4, 10000);

  // Both motors off for 2 seconds
  Serial.println("Both Motors OFF for 2 sec");
  digitalWrite(MOTOR1_IN1, LOW);
  digitalWrite(MOTOR1_IN2, LOW);
  digitalWrite(MOTOR2_IN3, LOW);
  digitalWrite(MOTOR2_IN4, LOW);
  delay(1000);

}
