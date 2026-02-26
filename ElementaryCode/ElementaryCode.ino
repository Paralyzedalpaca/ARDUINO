// ==========================================
// PIN DEFINITIONS
// ==========================================
const int SLP_PIN1 = 7;
const int SLP_PIN2 = 6;
const int DIREC_A = 10;
const int PWM_A = 0;
const int DIREC_B = 9;
const int PWM_B = 2;

// ==========================================
// FLIGHT PARAMETERS
// ==========================================
float VI0 = 2.0;                                  // Constant thrust action voltage
float flapping_frequency = 20.0;                  // Flapping frequency in Hz
float cycle_period = 1000.0 / flapping_frequency; // Time of one flapping cycle (50ms)

// ==========================================
// TIMING VARIABLES
// ==========================================
unsigned long cycle_timer = 0;                    // Stores the time in the latest flapping cycle
int cycle_counter = 0;                            // Tracks flapping cycles for the ramp-up phase

void setup() {
  // 1. Configure motor driver pins as outputs
  pinMode(SLP_PIN1, OUTPUT);
  pinMode(SLP_PIN2, OUTPUT);
  pinMode(DIREC_A, OUTPUT);
  pinMode(PWM_A, OUTPUT);
  pinMode(DIREC_B, OUTPUT);
  pinMode(PWM_B, OUTPUT);

  // 2. Set initial hardware states (Sleep pins HIGH to wake driver, PWM to 0)
  digitalWrite(SLP_PIN1, HIGH);
  digitalWrite(SLP_PIN2, HIGH);
  digitalWrite(DIREC_A, LOW);
  analogWrite(PWM_A, 0);
  digitalWrite(DIREC_B, LOW);
  analogWrite(PWM_B, 0);

  // 3. Initialize the cycle timer
  cycle_timer = millis();
}

void loop() {
  // Continuously apply the voltage sequence to the motors
  set_voltage();
}

// --------------------------------------------------------------
// Function to keep track of the number of passed flapping cycles
// --------------------------------------------------------------
void update_cycle() {
  if (millis() - cycle_timer > cycle_period) {
    cycle_timer = millis();
    cycle_counter += 1;
  }
}

// -------------------------------------------------------
// Function that applies the desired voltage to the motors
// -------------------------------------------------------
void set_voltage() {
  update_cycle();
  
  int volt_A = 0;
  int volt_B = 0;
  digitalWrite(DIREC_A, LOW);
  digitalWrite(DIREC_B, HIGH);
  volt_A = min((int)(VI0/3.7*255), (int)(cycle_counter/20.0*(VI0/3.7*255)));
  volt_B = min((int)(VI0/3.7*255), (int)(cycle_counter/20.0*(VI0/3.7*255)));
  /*
  // Alternate motor directions halfway through the cycle period
  if (millis() - cycle_timer > cycle_period * 0.5) {
    digitalWrite(DIREC_A, LOW);
    digitalWrite(DIREC_B, HIGH);
    
    // Ramp up voltage for the first 20 cycles
    if (cycle_counter < 20) {
      volt_A = min((int)(VI0/3.7*255), (int)(cycle_counter/20.0*(VI0/3.7*255)));
      volt_B = min((int)(VI0/3.7*255), (int)(cycle_counter/20.0*(VI0/3.7*255)));
    } else {
      volt_A = min((int)((VI0)/3.7*255), (int)255);
      volt_B = min((int)((VI0)/3.7*255), (int)255);
    }
  } else {
    digitalWrite(DIREC_A, HIGH);
    digitalWrite(DIREC_B, LOW);
    
    // Ramp up voltage for the first 20 cycles
    if (cycle_counter < 20) {
      volt_A = min((int)(VI0/3.7*255), (int)(cycle_counter/20.0*(VI0/3.7*255)));
      volt_B = min((int)(VI0/3.7*255), (int)(cycle_counter/20.0*(VI0/3.7*255)));
    } else {
      volt_A = min((int)((VI0)/3.7*255), (int)255);
      volt_B = min((int)((VI0)/3.7*255), (int)255);
    }
  }
  */
  // Send the calculated PWM signals to the motor driver
  analogWrite(PWM_A, volt_A);
  analogWrite(PWM_B, volt_B);
}
