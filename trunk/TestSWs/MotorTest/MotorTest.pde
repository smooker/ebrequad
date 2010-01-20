/*
Test code to perform a test of the Brushless Motor controller. The goal of the
code is:
- Test the response of the Controller.
- Test the maximin refresh speed of the controller.
- Test differencies between AnalogWrite and Servo library.

*/

//=============================================================================
//
// Global Definitions
//
//=============================================================================

#define LED_PIN             13
#define THROTTLE_PIN        50

//=============================================================================
//
// Global Variables
//
//=============================================================================

unsigned char state=0;
unsigned char counter = 0;

//=============================================================================
//
// Global functions 
//
//=============================================================================

void Motor_Genertate_Pulse(unsigned int period, unsigned int pulse);


//=============================== setup() =====================================
//
// Arduino Initialization
//
//=============================================================================

void setup()
{
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  pinMode(THROTTLE_PIN, OUTPUT);

  Serial.println("ESC Throttle Tests");

  digitalWrite(LED_PIN, HIGH);
  delay(200);
  digitalWrite(LED_PIN, LOW);
}


//=============================== loop() ======================================
//
// Arduino Main Loop Body
//
//=============================================================================



void Motor_Genertate_Pulse(unsigned int period, unsigned int pulse)
{
        delayMicroseconds(period-pulse);
        digitalWrite(THROTTLE_PIN, HIGH);
        delayMicroseconds(pulse);
        digitalWrite(THROTTLE_PIN, LOW);
}
    
void loop ()
{
    unsigned char c_rx, c_rx2;

    
    switch (state)
    {
        case 'a':
          Motor_Genertate_Pulse(5000,1000);
        break;
        case 'b':
          Motor_Genertate_Pulse(5000,1300);
        break;
        case 'c':
          if (counter < 20)
          {
            digitalWrite(LED_PIN, HIGH);
            Motor_Genertate_Pulse(5000,1300);
            counter++;
          }
          else if (counter < 40)
          {
            digitalWrite(LED_PIN, LOW);
            Motor_Genertate_Pulse(5000,1500);
            counter++;
          }
          else
          {
            counter = 0;
          }
        break;
        case 'd':
          Motor_Genertate_Pulse(4000,1300);
        break;
        case 'e':
          if (counter < 250)
          {
            Motor_Genertate_Pulse(5000,1300);
            counter++;
          }
          else if (counter < 252)
          {
            Motor_Genertate_Pulse(5000,2000);
            counter++;
          }
          else
          {
            counter = 0;
          }
        break;

        
        case 'f':
          if (counter < 250)
          {
            Motor_Genertate_Pulse(5000,1300);
            counter++;
          }
          else if (counter < 251)
          {
            Motor_Genertate_Pulse(5000,2000);
            counter++;
          }
          else if (counter < 261)
          {
            Motor_Genertate_Pulse(5000,1300);
            counter++;
          }
          else if (counter < 262)
          {
            Motor_Genertate_Pulse(5000,2000);
            counter++;
          }
          else
          {
            counter = 0;
          }
        break;
        default:
            /* do nothing, wait for new command */
        break;
    }

    /* Check for new commands */
    if (Serial.available())
    {
        c_rx = Serial.read();
        switch (c_rx)
        {
            case 'a':
                state = 'a';
                Serial.println("Test A: 5ms, pulse 1ms");
            break;
            case 'b':
                state = 'b';
                Serial.println("Test B: 5ms, pulse 1,3ms");
            break;
            case 'c':
                state = 'c';
                Serial.println("Test C: 5ms, pulse variable 100ms");
                counter = 0;
            break;
            case 'd':
                state = 'd';
                Serial.println("Test D: 4ms, pulse 1,3ms");
                counter = 0;
            break;
            case 'e':
                state = 'e';
                Serial.println("Test e: two pulses of 2ms every 1s");
                counter = 0;
            break;
            case 'f':
                state = 'f';
                Serial.println("Test f: ten pulses of 2ms every 1s");
                counter = 0;
            break;
            case 'A':
                /* Pending: AnalogWrite tests */
            break;
            case 'B':
                /* Pending: Servo tests */
            break;
            default:
                state = 255;
            break;
        }
    }
}

