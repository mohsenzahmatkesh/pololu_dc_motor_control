#define LEFT_FORWARD   4
#define LEFT_BACKWARD  18
#define LEFT_ENCODER_PIN 35

#define SWITCH1_PIN 22
#define SWITCH2_PIN 23

//Feedback
volatile long Left_counter = 0;

// Control params
float Kp = 0.8;  // Start with 0.5-1.0
float Ki = 0.1;  // Start with 0.05-0.2
int Target_Counts_left = 50;  


float integral_left = 0;
unsigned long last_time = 0;
const int interval = 100;  // 100ms control period
const float integral_limit = 500; 

// switches
bool reverse_direction = false;
bool last_switch_state = HIGH;

void setup() {
  Serial.begin(115200);
  while(!Serial);  
  
  pinMode(LEFT_FORWARD, OUTPUT);
  pinMode(LEFT_BACKWARD, OUTPUT);
  pinMode(LEFT_ENCODER_PIN, INPUT_PULLUP);
  pinMode(SWITCH1_PIN, INPUT_PULLUP);
  pinMode(SWITCH2_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_PIN), []{Left_counter++;}, RISING);

  Serial.println("PI Controller Ready");
}

int lookup_pwm(int target_counts) {
  // Define lookup tables
  const int target_table[] = {10, 20, 30, 40, 50, 100, 150};
  const int pwm_table[]    = {24, 30, 33, 39, 41, 55, 77};
  const int table_size = sizeof(target_table) / sizeof(target_table[0]);

  // If below minimum
  if (target_counts <= target_table[0]) return pwm_table[0];
  // If above maximum
  if (target_counts >= target_table[table_size - 1]) return pwm_table[table_size - 1];

  // Find correct interval and interpolate
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

  bool current_switch_state = digitalRead(SWITCH1_PIN);
  if (last_switch_state == HIGH && current_switch_state == LOW) {
    reverse_direction = !reverse_direction;  // Toggle direction
    delay(200);  // Basic debounce
  }
  last_switch_state = current_switch_state;

  
  if(now - last_time >= interval) {
  
    bool switch1_state = digitalRead(SWITCH1_PIN) == LOW;
    bool switch2_state = digitalRead(SWITCH2_PIN) == LOW;

   
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

    if (actual_left == 0) {
      int lookup_value = lookup_pwm(Target_Counts_left);
      pwm_left = lookup_value;
    }


    // analogWrite(LEFT_FORWARD, 0);
    // analogWrite(LEFT_BACKWARD, pwm_left);

    if (reverse_direction) {
      analogWrite(LEFT_FORWARD, pwm_left);
      analogWrite(LEFT_BACKWARD, 0);
    } else {
      analogWrite(LEFT_FORWARD, 0);
      analogWrite(LEFT_BACKWARD, pwm_left);
    }

    Serial.print("Target: ");
    Serial.print(Target_Counts_left);
    Serial.print(" | Actual Left: ");
    Serial.print(actual_left);
    Serial.print(" | PWM Left: ");
    Serial.println(pwm_left); // <-- use println here to go to next line


    last_time = now;
  }
}
