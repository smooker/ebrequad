/* Arduino Mega / MicroMag 3-Axis v1.0 Compass Example
// Uses native SPI support in Atmgea 1280
// Adapted from: http://forum.sparkfun.com/viewtopic.php?p=27072&sid=17c72d5a264f383836578d728cb60881#27072
// Ted Carancho - www.AeroQuad.info
*/

#include "Motors_api.h"



void setup()
{
  Serial.begin(115200);
  pinMode(13,OUTPUT);
  Motors_Init();
  Serial.println("Motors Initialization Done.");
  Serial.println("Waiting CMD...");
} 

void loop()
{
    static unsigned int comand=10; 
    static unsigned int comand_old=0;
    unsigned char incomingByte;
    static unsigned char fast_refresh=1;
    static unsigned char cyclic_change=0;
    static unsigned char toggle=1;
    static unsigned char delay_ms = 0;
    
    static unsigned int comands[4];

    digitalWrite(13, HIGH);

 /*   if (fast_refresh)
    {
        if (cyclic_change == 1)
        {
            if (toggle)
            {
                comand = comand + 800;
                toggle = 0;
            }
            else
            {
                comand = comand - 800;
                toggle = 1;
            }    
        }
        Motors_ControlAll(comand);
        comand_old = comand;
    }
    else
    {
        if (comand != comand_old)
        {
            Motors_ControlAll(comand);
            comand_old = comand;
        }
    }
*/
    
    if (delay_ms != 0)
    {
      delay((unsigned long)delay_ms);
    }
    
    digitalWrite(13, LOW);

    
    if (Serial.available())
    {
        //while(!Serial.available());
        incomingByte = Serial.read();
    
        switch(incomingByte)
        {
            case 'z': comand = 0;
            break;
            case 'i':
               Serial.print("-> Motors ");
               Serial.println(comand,DEC);
               Serial.println(delay_ms,DEC);
               Serial.println(cyclic_change,DEC);
               Serial.println("Waiting CMD...");
            break;
            case '+':
               comand++;
               delay_ms++;
            break;
            case '-':
               comand--;
               delay_ms--;
            break;  
            case '*':
               comand = comand + 100;
            break;
            case '/':
               comand = comand - 100;
            break;  
            case 'r':
                fast_refresh = 0;
            break;
            case 's':
                fast_refresh = 1;
            break;
            case 'y':
                if (cyclic_change){
                 cyclic_change = 0;}
                else{
                 cyclic_change = 1;}
            break;
            case 'a':
              comands[0] = 50;
              comands[1] = 250;
              comands[2] = 600;
              comands[3] = 1000;
              Motors_Control(&comands[0]);
            break;
            case 'b':
              comands[3] = 50;
              comands[2] = 250;
              comands[1] = 600;
              comands[0] = 1000;
              Motors_Control(&comands[0]);
            break;
            case 'c':
              comands[1] = 50;
              comands[3] = 250;
              comands[0] = 600;
              comands[2] = 1000;
              Motors_Control(&comands[0]);
            break;
        }
    }
}

