#define RIGHT_FORWARD  19
#define RIGHT_BACKWARD 21
#define LEFT_FORWARD   4
#define LEFT_BACKWARD  18
#define RIGHT_ENCODER_PIN 34  
#define LEFT_ENCODER_PIN 35   

#define mot_mir_vrt_a1  0
#define mot_mir_vrt_a2 4
#define mot_cam_hrz_b1   2
#define mot_cam_hrz_b2  15
#define sen_mir_vrt_vel_a 35  
#define sen_mir_vrt_vel_b 32  
#define sen_cam_vel_a 33   
#define sen_cam_vel_b 25

volatile long Right_counter = 0;
volatile long Left_counter = 0;


float Kp = 0.8;  
float Ki = 0.1;  
int Target_Counts_left = 10;  
int Target_Counts_right = 10;


float integral_right = 0;
float integral_left = 0;
unsigned long last_time = 0;
const int interval = 100;  // 100ms control period
const float integral_limit = 500; 

void setup() {
  Serial.begin(115200);
  while(!Serial);  // Wait for serial connection
  

  pinMode(RIGHT_FORWARD, OUTPUT);
  pinMode(RIGHT_BACKWARD, OUTPUT);
  pinMode(LEFT_FORWARD, OUTPUT);
  pinMode(LEFT_BACKWARD, OUTPUT);


  pinMode(RIGHT_ENCODER_PIN, INPUT_PULLUP);
  pinMode(LEFT_ENCODER_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_PIN), []{Right_counter++;}, RISING);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_PIN), []{Left_counter++;}, RISING);

  pinMode(mot_mir_vrt_a1, OUTPUT);
  pinMode(mot_mir_vrt_a2, OUTPUT);
  pinMode(mot_cam_hrz_b1, OUTPUT);
  pinMode(mot_cam_hrz_b2, OUTPUT);


  pinMode(sen_mir_vrt_vel_a, INPUT_PULLUP);
  pinMode(sen_cam_vel_a, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(sen_mir_vrt_vel_a), []{Right_counter++;}, RISING);
  attachInterrupt(digitalPinToInterrupt(sen_cam_vel_a), []{Left_counter++;}, RISING);


  Serial.println("PI Controller Ready");
}

void loop() {
  unsigned long now = millis();
  
  if(now - last_time >= interval) {
   
    noInterrupts();
    int actual_right = Right_counter;
    int actual_left = Left_counter;
    Right_counter = 0;
    Left_counter = 0;
    interrupts();

    int error_right = Target_Counts_right - actual_right;
    int error_left = Target_Counts_left - actual_left;

    integral_right += error_right;
    integral_left += error_left;

    int pwm_right = Kp * error_right + Ki * integral_right;
    int pwm_left = Kp * error_left + Ki * integral_left;

    int constrained_right = constrain(pwm_right, 0, 255);
    int constrained_left = constrain(pwm_left, 0, 255);

    if(pwm_right == constrained_right) {
      integral_right += error_right;
    } else {

      // Clamp integral when saturated

      integral_right = constrain(integral_right, -integral_limit, integral_limit);
    }

    if(pwm_left == constrained_left) {
      integral_left += error_left;
    } else {
      integral_left = constrain(integral_left, -integral_limit, integral_limit);
    }


    pwm_right = constrained_right;
    pwm_left = constrained_left;


    // pwm_right = constrain(pwm_right, 0, 255);
    // pwm_left = constrain(pwm_left, 0, 255);



    analogWrite(RIGHT_FORWARD, 0);
    analogWrite(RIGHT_BACKWARD, pwm_right);
    analogWrite(LEFT_FORWARD, 0);
    analogWrite(LEFT_BACKWARD, pwm_left);

    analogWrite(mot_mir_vrt_a1, 0);
    analogWrite(mot_mir_vrt_a2, pwm_right);
    analogWrite(mot_cam_hrz_b1, 0);
    analogWrite(mot_cam_hrz_b2, pwm_left);



    Serial.print("Target: "); Serial.print(Target_Counts_right);
    Serial.print(" | Actual R: "); Serial.print(actual_right);
    Serial.print(" L: "); Serial.print(actual_left);
    Serial.print(" | PWM R: "); Serial.print(pwm_right);
    Serial.print(" L: "); Serial.println(pwm_left);

    last_time = now;
  }
}
