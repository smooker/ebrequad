// define spi bus pins
#define SLAVESELECT 47
#define DRDY 46
#define SPICLOCK 52
#define DATAOUT 51	//MOSI
#define DATAIN 50	 //MISO
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
  pinMode(53, OUTPUT);
  pinMode(DRDY, INPUT);
  pinMode(SLAVESELECT,OUTPUT);
  digitalWrite(53, HIGH);
  digitalWrite(SLAVESELECT,HIGH); //disable device  
  
  SPCR = B01010011; //MPIE=0, SPE=1 (on), DORD=0 (MSB first), MSTR=1 (master), CPOL=0 (clock idle when low), CPHA=0 (samples MOSI on rising edge), SPR1=0 & SPR0=0 (500kHz)
  clr=SPSR;
  clr=SPDR;
  Serial.begin(115200);
  delay(500);

  Serial.println("Initialize High Speed Constant Reading Mode");
  write_register(0x02, 0x2D);
  write_register(0x01, 0x03);
  write_register(0x03, 0x02);
  delay(1000);
  write_register(0x03,0x09);
}

void loop()
{
  //rev_in_byte = read_register(REVID);
  
  temp_in = read_register16(TEMP);
  temp_in = temp_in * 0.05;
  //temp_in = ((1.8) *temp_in) + 32;

  pressure_msb = read_register(PRESSURE);
  pressure_msb &= B00000111;
  pressure_lsb = read_register16(PRESSURE_LSB);
  pressure = UBLB19(pressure_msb, pressure_lsb);
  pressure = pressure * 0.25;
  
  Serial.print("PRESSURE [");
  Serial.print(pressure, DEC);
  Serial.println("]");
  
  Serial.print("TEMP C [");
  Serial.print(temp_in , DEC);
  Serial.println("]");

  while (digitalRead(DRDY) == LOW);
  //delay(1500);
  
}

char spi_transfer(volatile char data)
{
  SPDR = data;			  // Start the transmission
  while (!(SPSR & (1<<SPIF)))     // Wait for the end of the transmission
  {
  };
  return SPDR;			  // return the received byte
}


char read_register(byte register_name)
{
    char in_byte;
    register_name <<= 2;
    register_name &= B11111100; //Read command
  
    digitalWrite(SLAVESELECT,LOW); //Select SPI Device
    spi_transfer(register_name); //Write byte to device
    in_byte = spi_transfer(0x00); //Send nothing, but we should get back the register value
    digitalWrite(SLAVESELECT,HIGH);
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
