/*
Test code to log the analog signal from 5DOF Gyros and Accelerometer.
The goal of the code is:
- Be able to log the raw sensor data and perform excel analsys and code simulation.
*/

//=============================================================================
//
// Global Definitions
//
//=============================================================================

#define LED_PIN             13
#define AZPIN               12

// Sensor pin assignments
#define PITCHACCELPIN 0
#define ROLLACCELPIN 1
#define ZACCELPIN 2
#define PITCHRATEPIN 3
#define ROLLRATEPIN 4


//=============================================================================
//
// Global Variables
//
//=============================================================================

unsigned char state=0;
unsigned int delay_var = 9500;

//=============================================================================
//
// Global functions 
//
//=============================================================================




//=============================== setup() =====================================
//
// Arduino Initialization
//
//=============================================================================

void setup()
{
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  pinMode(AZPIN,   OUTPUT);

  Serial.println("-- Sensor Raw Data Logging --");

  analogReference(EXTERNAL); // Current external ref is connected to 3.3V

  digitalWrite(LED_PIN, HIGH);
  autoZeroGyros();
  delay(100);
  digitalWrite(LED_PIN, LOW);
}


//=============================== loop() ======================================
//
// Arduino Main Loop Body
//
//=============================================================================

void loop ()
{
    unsigned char c_rx, c_rx2;
    unsigned int data0, data1, data2;
    
    switch (state)
    {
        case 'p': /* log Pitch */
            digitalWrite(LED_PIN, HIGH);
            data0 = analogRead(PITCHACCELPIN);
            data1 = analogRead(ZACCELPIN);
            data2 = analogRead(PITCHRATEPIN);
            digitalWrite(LED_PIN, LOW);
            Serial.print(data0, DEC); Serial.print(",");
            Serial.print(data1, DEC); Serial.print(",");
            Serial.println(data2, DEC);
            delayMicroseconds(delay_var);
        break;
        
        case 'r': /* log Roll */
            digitalWrite(LED_PIN, HIGH);
            data0 = analogRead(ROLLACCELPIN);
            data1 = analogRead(ZACCELPIN);
            data2 = analogRead(ROLLRATEPIN);
            digitalWrite(LED_PIN, LOW);
            Serial.print(data0, DEC); Serial.print(",");
            Serial.print(data1, DEC); Serial.print(",");
            Serial.println(data2, DEC);
            delayMicroseconds(delay_var);
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
            case 'p':
                if (state == 0)
                {
                    Serial.println("-- Start logging Pitch: PitchACC, ZACC, PitchGY --");
                    Serial.print(" Start time:"); Serial.println(milis(),DEC);
                    state = 'p';
                }
                else if (state == 'p')
                {
                    Serial.print(" End time:"); Serial.println(milis(),DEC);
                    Serial.println("-- Stop logging Pith --");
                    state = 0;
                }
            break;
            case 'r':
                if (state == 0)
                {
                    Serial.println("-- Start logging Roll: RollACC, ZACC, RollGY --");
                    Serial.print(" Start time:"); Serial.println(milis(),DEC);
                    state = 'r';
                }
                else if (state == 'r')
                {
                    Serial.print(" End time:"); Serial.println(milis(),DEC);
                    Serial.println("-- Stop logging Roll --");
                    state = 0;
                }
            break;
            case 'z': /* reset Gyro */
                autoZeroGyros();
            break;
            case '+':
                delay_var = delay_var + 100;
                Serial.print("New Delay: ");Serial.println(delay_var, DEC);
            break;
            case '-':
                delay_var = delay_var - 100;
                Serial.print("New Delay: ");Serial.println(delay_var, DEC);
            break;
            default:
                state = 0;
            break;
        }
    }
}

void autoZeroGyros(void)
{
  digitalWrite(AZPIN, HIGH);
  delayMicroseconds(750);
  digitalWrite(AZPIN, LOW);
  delay(8);
}

