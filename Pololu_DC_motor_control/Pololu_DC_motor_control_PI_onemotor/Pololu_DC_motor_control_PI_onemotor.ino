#define LEFT_FORWARD   4
#define LEFT_BACKWARD  18
#define LEFT_ENCODER_PIN 35   

volatile long Left_counter = 0;


float Kp = 0.8;  // Start with 0.5-1.0
float Ki = 0.1;  // Start with 0.05-0.2
int Target_Counts_left = 150;  


float integral_left = 0;
unsigned long last_time = 0;
const int interval = 100;  // 100ms control period
const float integral_limit = 500; 

void setup() {
  Serial.begin(115200);
  while(!Serial);  
  
  pinMode(LEFT_FORWARD, OUTPUT);
  pinMode(LEFT_BACKWARD, OUTPUT);


  pinMode(LEFT_ENCODER_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_PIN), []{Left_counter++;}, RISING);

  Serial.println("PI Controller Ready");
}

void loop() {
  unsigned long now = millis();
  
  if(now - last_time >= interval) {
   
    noInterrupts();
    int actual_left = Left_counter;
    Left_counter = 0;
    interrupts();

    int error_left = Target_Counts_left - actual_left;

    integral_left += error_left;

    int pwm_left = Kp * error_left + Ki * integral_left;

    int constrained_left = constrain(pwm_left, 0, 255);


    if(pwm_left == constrained_left) {
      integral_left += error_left;
    } else {
      integral_left = constrain(integral_left, -integral_limit, integral_limit);
    }

    pwm_left = constrained_left;

    // // Constrain PWM outputs
    // pwm_right = constrain(pwm_right, 0, 255);
    // pwm_left = constrain(pwm_left, 0, 255);

    analogWrite(LEFT_FORWARD, 0);
    analogWrite(LEFT_BACKWARD, pwm_left);

    Serial.print("Target: ");
    Serial.print(Target_Counts_left);
    Serial.print(" | Actual Left: ");
    Serial.print(actual_left);
    Serial.print(" | PWM Left: ");
    Serial.println(pwm_left); // <-- use println here to go to next line


    last_time = now;
  }
}