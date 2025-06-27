#define mot_mir_vrt_a1  0
#define mot_mir_vrt_a2 4
#define mot_cam_hrz_b1   2
#define mot_cam_hrz_b2  15
#define sen_mir_vrt_vel_a 35  
#define sen_mir_vrt_vel_b 32  
#define sen_cam_vel_a 33   
#define sen_cam_vel_b 25
//#define sw_mir_top_l 13
//#define sw_mir_bot_l 9

#define SWITCH1_PIN 22
#define SWITCH2_PIN 23

//Feedback
volatile long sen_mir_vrt_vel_a_count = 0;
volatile long sen_cam_vel_a_count = 0;

// Control params
float Kp = 0.9;  // Start with 0.5-1.0
float Ki = 0.1;  // Start with 0.05-0.2
int Target_Counts_left = 100;  


float sen_mir_vrt_vel_a_integral = 0;
float sen_cam_vel_a_integral = 0;
unsigned long last_time = 0;
const int interval = 100;  // 100ms control period
const float integral_limit = 500; 

// switches
bool reverse_direction = false;
bool last_switch1_state = HIGH;
bool last_switch2_state = HIGH;

void setup() {
  Serial.begin(115200);
  while(!Serial);  

  pinMode(mot_mir_vrt_a1, OUTPUT);
  pinMode(mot_mir_vrt_a2, OUTPUT);
  pinMode(mot_cam_hrz_b1, OUTPUT);
  pinMode(mot_cam_hrz_b2, OUTPUT);
  pinMode(sen_mir_vrt_vel_a, INPUT_PULLUP );
  pinMode(sen_cam_vel_a, INPUT_PULLUP);
  pinMode(SWITCH1_PIN, INPUT_PULLUP);
  pinMode(SWITCH2_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(sen_mir_vrt_vel_a), []{sen_mir_vrt_vel_a_count++;}, RISING);
  attachInterrupt(digitalPinToInterrupt(sen_cam_vel_a), []{sen_cam_vel_a_count++;}, RISING);

  Serial.println("PI Controller Ready");
}

int lookup_pwm(int target_counts) {
  // Define lookup tables
  const int target_table[] = {10, 20, 30, 40, 50, 100, 150};
  const int pwm_table[]    = {24, 30, 33, 39, 41, 55, 77};
  const int table_size = sizeof(target_table) / sizeof(target_table[0]);

  // Saturation
  if (target_counts <= target_table[0]) return pwm_table[0];
  if (target_counts >= target_table[table_size - 1]) return pwm_table[table_size - 1];

  // Interpolate
  for (int i = 0; i < table_size - 1; i++) {
    if (target_counts >= target_table[i] && target_counts <= target_table[i + 1]) {
      float ratio = (float)(target_counts - target_table[i]) / (target_table[i + 1] - target_table[i]);
      return pwm_table[i] + ratio * (pwm_table[i + 1] - pwm_table[i]);
    }
  }

  return -1; // Should not happen
}


void loop() {
  unsigned long now = millis();

  if (Serial.available()) {
  char cmd = Serial.read();
  if (cmd == 'F') reverse_direction = false;
  else if (cmd == 'R') reverse_direction = true;
  }

  bool current_switch_state = digitalRead(SWITCH1_PIN);
  if (last_switch1_state == HIGH && current_switch_state == LOW) {
    reverse_direction = !reverse_direction;  // Toggle direction
    delay(200);  // Basic debounce
  }
  last_switch1_state = current_switch_state;

  bool current_switch2_state = digitalRead(SWITCH2_PIN);
  if (last_switch2_state == HIGH && current_switch2_state == LOW) {
    Target_Counts_left = (Target_Counts_left == 80) ? 100 : 80;  // Toggle between 50 and 100
    delay(200);  // Debounce
  }
  last_switch2_state = current_switch2_state;

  
  if(now - last_time >= interval) {
  
    bool switch1_state = digitalRead(SWITCH1_PIN) == LOW;
    bool switch2_state = digitalRead(SWITCH2_PIN) == LOW;

   
    noInterrupts();
    int sen_mir_vrt_vel_a_actual = sen_mir_vrt_vel_a_count;
    sen_mir_vrt_vel_a_count = 0;
    int sen_cam_vel_a_actual = sen_cam_vel_a_count;
    sen_cam_vel_a_count = 0;
    interrupts();

    int sen_mir_vrt_vel_a_error = Target_Counts_left - sen_mir_vrt_vel_a_actual;
    int sen_cam_vel_a_error = Target_Counts_left - sen_cam_vel_a_actual;

    // MIRROR
    sen_mir_vrt_vel_a_integral += sen_mir_vrt_vel_a_error;
    int sen_mir_vrt_vel_a_pwm = Kp * sen_mir_vrt_vel_a_error + Ki * sen_mir_vrt_vel_a_integral;
    int sen_mir_vrt_vel_a_constraint = constrain(sen_mir_vrt_vel_a_pwm, 0, 255);
    if (sen_mir_vrt_vel_a_pwm == sen_mir_vrt_vel_a_constraint) {
      sen_mir_vrt_vel_a_integral += sen_mir_vrt_vel_a_error;
    } else {
      sen_mir_vrt_vel_a_integral = constrain(sen_mir_vrt_vel_a_integral, -integral_limit, integral_limit);
    }
    sen_mir_vrt_vel_a_pwm = sen_mir_vrt_vel_a_constraint;
    if (sen_mir_vrt_vel_a_actual == 0) {
      int lookup_value = lookup_pwm(Target_Counts_left);
      sen_mir_vrt_vel_a_pwm = lookup_value;
    }

    // CAMERA
    sen_cam_vel_a_integral += sen_cam_vel_a_error;
    int sen_cam_vel_a_pwm = Kp * sen_cam_vel_a_error + Ki * sen_cam_vel_a_integral;
    int sen_cam_vel_a_constraint = constrain(sen_cam_vel_a_pwm, 0, 255);
    if (sen_cam_vel_a_pwm == sen_cam_vel_a_constraint) {
      sen_cam_vel_a_integral += sen_cam_vel_a_error;
    } else {
      sen_cam_vel_a_integral = constrain(sen_cam_vel_a_integral, -integral_limit, integral_limit);
    }
    sen_cam_vel_a_pwm = sen_cam_vel_a_constraint;
    if (sen_cam_vel_a_actual == 0) {
      int lookup_value = lookup_pwm(Target_Counts_left);
      sen_cam_vel_a_pwm = lookup_value;
    }


    // analogWrite(mot_cam_hrz_b1, 0);
    // analogWrite(mot_cam_hrz_b2, pwm_left);

    // MIRROR MOTOR CONTROL
    if (reverse_direction) {
      analogWrite(mot_mir_vrt_a1, sen_mir_vrt_vel_a_pwm);  // Assuming pin names
      analogWrite(mot_mir_vrt_a2, 0);
    } else {
      analogWrite(mot_mir_vrt_a1, 0);
      analogWrite(mot_mir_vrt_a2, sen_mir_vrt_vel_a_pwm);
    }

    // CAMERA MOTOR CONTROL
    if (reverse_direction) {
      analogWrite(mot_cam_hrz_b1, sen_cam_vel_a_pwm);
      analogWrite(mot_cam_hrz_b2, 0);
    } else {
      analogWrite(mot_cam_hrz_b1, 0);
      analogWrite(mot_cam_hrz_b2, sen_cam_vel_a_pwm);
    }


    Serial.print("Target: ");
    Serial.print(Target_Counts_left);
    Serial.print(" | Mirror Actual: ");
    Serial.print(sen_mir_vrt_vel_a_actual);
    Serial.print(" | Mirror PWM: ");
    Serial.print(sen_mir_vrt_vel_a_pwm);
    Serial.print(" | Camera Actual: ");
    Serial.print(sen_cam_vel_a_actual);
    Serial.print(" | Camera PWM: ");
    Serial.println(sen_cam_vel_a_pwm);  // Ends line



    last_time = now;
  }
}
