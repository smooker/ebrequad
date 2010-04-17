//It is a 3.3 V device, so if you are using a 5v arduino to power it that might be the problem. I am using the voltage and ground from a //Sparkfun wee (lillypad equivalent). Other than that, pins 10-13 for ss, sck, mosi, miso.

//MOSI, MISO, and SCK are buses, but the chip select is unique for each device.  If the device is a correctly designed SPI device, just wire the //second one up identical to the first, except use a different line for chip select
//CSB should be connected to SLAVESELECT.

float tempPressure;           // for raw sensor values 
float filterVal;       // this determines smoothness  - .0001 is max  1 is off (no smoothing)
float smoothedVal;     // this holds the last loop value just use a unique variable for every different sensor that needs smoothing
float smoothedAlt;
float tempAltitude;
int i, j;              // loop counters or demo

float Pb = 101888;
float Tb = 288.15;
float Lb = -0.0065;
float hb = 0;
float g0 = 9.80665;
float M = 0.0289644;
float R = 8.31432;
float altitude = 0;

// h = [Tb / (P / Pb) ^ ((R * Lb) / (g0 * M)) - Tb] / (Lb + hb)
// h = Pb * (Tb / (Tb + Lb * (0 - 0))) ^ ((g0 * M) / (R * Lb))


// define spi bus pins
#define SLAVESELECT 47
#define SPICLOCK 52
#define DATAOUT 51	//MOSI
#define DATAIN 50	 //MISO
#define DRDY 46
#define UBLB(a,b)  ( ( (a) << 8) | (b) )
#define UBLB19(a,b) ( ( (a) << 16 ) | (b) )

//Addresses
#define REVID 0x00	//ASIC Revision Number
#define OPSTATUS 0x04   //Operation Status
#define STATUS 0x07     //ASIC Status
#define START 0x0A      //Constant Readings
#define PRESSURE 0x1F   //Pressure 3 MSB
#define PRESSURE_LSB 0x20 //Pressure 16 LSB
#define TEMP 0x21       //16 bit temp

char rev_in_byte;	    
int temp_in;
unsigned long pressure_lsb;
unsigned long pressure_msb;
unsigned long temp_pressure;
unsigned long pressure;



void setup()
{
  byte clr;
  pinMode(DATAOUT, OUTPUT);
  pinMode(DATAIN, INPUT);
  pinMode(SPICLOCK,OUTPUT);
  pinMode(SLAVESELECT,OUTPUT);
  pinMode(DRDY, INPUT);
  digitalWrite(SLAVESELECT,HIGH); //disable device  
  digitalWrite(53, HIGH); //physical SS pin high before setting SPCR

  SPCR = B01010011; //MPIE=0, SPE=1 (on), DORD=0 (MSB first), MSTR=1 (master), CPOL=0 (clock idle when low), CPHA=0 (samples MOSI on rising edge), SPR1=0 & SPR0=0 (500kHz)
  clr=SPSR;
  clr=SPDR;
  delay(1000);
  Serial.begin(115200);

  Serial.println("Initialize High Speed Constant Reading Mode");
  write_register(0x03,0x0A);
  filterVal =  .5; 
  pressure = 0;  
  rev_in_byte = read_register(REVID);
  i = 0;
}


void loop()
{
  
  while (digitalRead(DRDY) == LOW);

  temp_in = read_register16(TEMP);
  //temp_in = temp_in * 0.05;
  //temp_in = ((1.8) *temp_in) + 32;

  pressure_msb = read_register(PRESSURE);
  pressure_msb &= B00000111;
  pressure_lsb = read_register16(PRESSURE_LSB);
  pressure = UBLB19(pressure_msb, pressure_lsb);
  //pressure /= 4;
  //pressure = pressure*4;
  if (i==0) {
    smoothedVal = pressure;
    i = 1;
  }
  
  smoothedVal =  smooth(pressure, filterVal, smoothedVal);   // second parameter determines smoothness  - 0 is off,  .9999 is max smooth 
  //Serial.println(16250-smoothedVal);
  //Serial.print(14280-smoothedVal); Serial.print("     ");
  Serial.print(smoothedVal); Serial.print("     ");
  
  tempPressure = pressure * 0.25;
  //altitude = map(tempPressure, 101352.784, 98319.0955, 0, 820);
  tempAltitude = map(tempPressure, 101840, 98806, 0, 820);
  altitude = smooth(tempAltitude, filterVal, smoothedAlt);
  Serial.print(tempPressure); Serial.print("     "); Serial.println(altitude);
}


char spi_transfer(volatile char data)
{
  SPDR = data;			  // Start the transmission
  while (!(SPSR & (1<<SPIF)))     // Wait for the end of the transmission
  {
  };
  return SPDR;			  // return the received byte
}


char read_register(char register_name)
{
    char in_byte;
    register_name <<= 2;
    register_name &= B11111100; //Read command
  
    digitalWrite(SLAVESELECT,LOW); //Select SPI Device
    spi_transfer(register_name); //Write byte to device
    in_byte = spi_transfer(0x00); //Send nothing, but we should get back the register value
    digitalWrite(SLAVESELECT,HIGH);
    delay(10);
    return(in_byte);
  
}

float read_register16(char register_name)
{
    byte in_byte1;
    byte in_byte2;
    float in_word;
    
    register_name <<= 2;
    register_name &= B11111100; //Read command

    digitalWrite(SLAVESELECT,LOW); //Select SPI Device
    spi_transfer(register_name); //Write byte to device
    in_byte1 = spi_transfer(0x00);    
    in_byte2 = spi_transfer(0x00);
    digitalWrite(SLAVESELECT,HIGH);
    in_word = UBLB(in_byte1,in_byte2);
    return(in_word);
}

void write_register(char register_name, char register_value)
{
    register_name <<= 2;
    register_name |= B00000010; //Write command

    digitalWrite(SLAVESELECT,LOW); //Select SPI device
    spi_transfer(register_name); //Send register location
    spi_transfer(register_value); //Send value to record into register
    digitalWrite(SLAVESELECT,HIGH);
} 



//========================================


float smooth(int data, float filterVal, float smoothedVal)
{
  return (data * (1 - filterVal)) + (smoothedVal  *  filterVal);
}


//======================================================
