#define RC_CHANS 6
#define PCINT_PIN_COUNT 6
#define PCINT_RX_BITS (1<<2),(1<<4),(1<<5),(1<<6),(1<<7),(1<<0)
#define RX_PCINT_PIN_PORT PIND
#define RX_PCINT_PIN_PORT2 PINB
#define MIN_PWM_IN 1000
#define MAX_PWM_IN 2000
#define ROLL_MID_PWM 1500
#define PITCH_MID_PWM 1500
#define YAW_MID_PWM 1500

// Put this in a folder with TK's "RC" file

static int16_t rcData[RC_CHANS];
static uint8_t PCInt_RX_Pins[PCINT_PIN_COUNT] = {PCINT_RX_BITS};
volatile uint16_t rcValue[RC_CHANS];
volatile int16_t RC_Command[RC_CHANS];
static uint16_t edgeTime[RC_CHANS];

uint16_t Thrust; uint16_t Roll; uint16_t Pitch; uint16_t Yaw; uint16_t Aux;
uint16_t Speed1; uint16_t Speed2; uint16_t Speed3; uint16_t Speed4;
uint16_t maxAngle = 30; // How much we want RollPitchYaw to affect motor speed, should turn this into 3 different values
uint16_t maxAngleInput = 500; // From hardware (controller)

enum rc {
  ROLL,
  PITCH,
  THR,
  YAW,
  AUX,
  AUX2
};

void setup() {
  initPCINT();
  Serial.begin(115200);

  DDRD |= (1 << DDD3); // PD3 Pin 3
  DDRB |= (1 << DDB3); // PB3 Pin 11
  DDRB |= (1 << DDB1); // PB1 Pin 9
  DDRB |= (1 << DDB2); // PD5 Pin 10

  TCCR2A |= (1 << COM2B1); TCCR2A &= ~(1 << COM2B0);
  TCCR2A |= (1 << COM2A1); TCCR2A &= ~(1 << COM2A0);
  TCCR2B &= ~(1 << WGM22); TCCR2A &= ~(1 << WGM21); TCCR2A |= (1 << WGM20);
  TCCR2B |= (1 << CS22); TCCR2B &= ~(1 << CS21); TCCR2B &= ~(1 << CS20);

  TCCR1A |= (1 << COM1B1); TCCR1A &= ~(1 << COM1B0);
  TCCR1A |= (1 << COM1A1); TCCR1A &= ~(1 << COM1A0);
  TCCR1B &= ~(1 << WGM12); TCCR1A &= ~(1 << WGM11); TCCR1A |= (1 << WGM10);
  TCCR1B &= ~(1 << CS12); TCCR1B |=(1 << CS11); TCCR1B |= (1 << CS10);

  delay(1000);

  OCR2B = 125; // PD3 Pin 3
  OCR2A = 125; // PB3 Pin 11
  OCR1A = 125; // PB1 Pin 9
  OCR1B = 125; // PB2 Pin 10
  
  delay(2000);
}

void loop() {
  computeRC();
  spinMotors(1); // Input 1 to see values, only if PrintRC is not in use
  //PrintRC();
}

// Roll, Pitch, Yaw are range (-500, 500)
// Thrust is range (1000, 2000)
// AUX is (1000, 1500, or 2000) +- 100
// AUX2 is (1000, 2000) +- 100

void spinMotors( int printer ) {
  // Get values from controller
  Thrust = RC_Command[THR]; Roll = RC_Command[ROLL];
  Pitch = RC_Command[PITCH]; Yaw = RC_Command[YAW];
  Aux = RC_Command[AUX];

  // Testing - comment out if you have a controller
  Aux = 1000;
  Thrust = 2000; Roll = 0;
  Pitch = 0; Yaw = 300;
  
  // Calculate spin speeds
  Speed1 = Thrust / 8; // Change Thrust to fit the correct range
  Speed2 = Thrust / 8;
  Speed3 = Thrust / 8;
  Speed4 = Thrust / 8;
  
  if (Yaw > 700) { // If Yaw is negative it rolls over to 65536
    Yaw = 65536 - Yaw; // Take absolute value
    Yaw = Yaw * maxAngle / maxAngleInput; // Change range
    Speed1 = Speed1 - Yaw; // Inverted change speeds accordingly
    Speed2 = Speed2 + Yaw;
    Speed3 = Speed3 + Yaw;
    Speed4 = Speed4 - Yaw;
  } else { // If Yaw is positive
    Yaw = Yaw * maxAngle / maxAngleInput;
    Speed1 = Speed1 + Yaw;
    Speed2 = Speed2 - Yaw;
    Speed3 = Speed3 - Yaw;
    Speed4 = Speed4 + Yaw;
  }
  if (Roll > 700) {
    Roll = 65536 - Roll;
    Roll = Roll * maxAngle / maxAngleInput;
    Speed1 = Speed1 - Roll;
    Speed2 = Speed2 + Roll;
    Speed3 = Speed3 - Roll;
    Speed4 = Speed4 + Roll;
  } else {
    Roll = Roll * maxAngle / maxAngleInput;
    Speed1 = Speed1 + Roll;
    Speed2 = Speed2 - Roll;
    Speed3 = Speed3 + Roll;
    Speed4 = Speed4 - Roll;
  }
  if (Pitch > 700) {
    Pitch = 65536 - Pitch;
    Pitch = Pitch * maxAngle / maxAngleInput;
    Speed1 = Speed1 - Pitch;
    Speed2 = Speed2 - Pitch;
    Speed3 = Speed3 + Pitch;
    Speed4 = Speed4 + Pitch;
  } else {
    Pitch = Pitch * maxAngle / maxAngleInput;
    Speed1 = Speed1 + Pitch;
    Speed2 = Speed2 + Pitch;
    Speed3 = Speed3 - Pitch;
    Speed4 = Speed4 - Pitch;
  }
  
  // The above calculations are based on these functions from a MATLAB drone tutorial on youtube
//  OCR2B = Thrust + Yaw + Pitch + Roll; //Front right - clockwise
//  OCR2A = Thrust - Yaw + Pitch - Roll; //Front left - counter clockwise
//  OCR1A = Thrust - Yaw - Pitch + Roll; //Back right - counter clockwise
//  OCR1B = Thrust + Yaw - Pitch - Roll; //Back left - clockwise

  // Killswitch - could be set to any switch/button
  if (Aux > 1200) {
     Thrust = 50; Yaw = 10; Pitch = 10; Roll = 10; // Low enough that the motors are off
  }

  if (Speed1 > 255) { Speed1 = 255; }
  if (Speed2 > 255) { Speed2 = 255; }
  if (Speed3 > 255) { Speed3 = 255; }
  if (Speed4 > 255) { Speed4 = 255; }

  OCR2B = Speed1; //Front right - clockwise
  OCR2A = Speed2; //Front left - counter clockwise
  OCR1A = Speed3; //Back right - counter clockwise
  OCR1B = Speed4; //Front left - clockwise
  
  if (printer == 1) {
    Serial.print("MOTORS: "); 
    Serial.print(OCR2B);  Serial.print(", "); 
    Serial.print(OCR2A); Serial.print(", "); 
    Serial.print(OCR1A);   Serial.print(", "); 
    Serial.print(OCR1B);

    Serial.print(" RC: "); 
    Serial.print(  Roll);  Serial.print(", "); 
    Serial.print(  Pitch); Serial.print(", "); 
    Serial.print(  Yaw);   Serial.print(", "); 
    Serial.println(Thrust);
  }
}

void PrintRC(){
  Serial.print(" RC: "); 
  Serial.print(  RC_Command[ROLL]);  Serial.print(", "); 
  Serial.print(  RC_Command[PITCH]); Serial.print(", "); 
  Serial.print(  RC_Command[THR]);   Serial.print(", "); 
  Serial.print(  RC_Command[YAW]);   Serial.print(", ");
  Serial.print(  RC_Command[AUX]);   Serial.print(", "); 
  Serial.println(RC_Command[AUX2]);
}
