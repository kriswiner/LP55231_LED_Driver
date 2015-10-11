/* LP55231 LED Square
 by: Kris Winer
 date: October 3, 2015
 license: Beerware - Use this code however you'd like. If you 
 find it useful you can buy me a beer some time.
 
 This sketch demonstrates the use of the TI LP55231 programmable LED driver to control 3 rgb leds on a 
 0.7 x 0.5 inch Teensy 3.1 add-on board.
 
 This sketch uses SDA/SCL on pins 17/16, respectively, and it uses the Teensy 3.1-specific Wire library i2c_t3.h.
 
 SDA and SCL have 4K7 pull-up resistors (to 3.3V).
 
 Hardware setup:
 LED Square----------- Teensy 3.1
 VDD ---------------------- 3.3V
 SDA ----------------------- 17
 SCL ----------------------- 16
 GND ---------------------- GND
 
 Note: All the sensors n this board are I2C sensor and uses the Teensy 3.1 i2c_t3.h Wire library. 
 The led drivers are 5V tolerant, but we are using a 3.3 V 8 MHz Pro Mini or a 3.3 V Teensy 3.1.
 */
 
#include <i2c_t3.h>
#include <SPI.h>

// LP55231 register Map, see :http://www.eccn.com/uploads/solution/201308/20130826142008846.pdf

#define ENABLE_ENGINE_CNTRL1    0x00
#define ENGINE_CNTRL2           0x01
#define OUTPUT_DIRECT_MSB       0x02
#define OUTPUT_DIRECT_LB        0x03
#define OUTPUT_ON_OFF_CNTL_MSB  0x04
#define OUTPUT_ON_OFF_CNTL_LSB  0x05
#define D1_CONTROL              0x06
#define D2_CONTROL              0x07
#define D3_CONTROL              0x08
#define D4_CONTROL              0x09
#define D5_CONTROL              0x0A
#define D6_CONTROL              0x0B
#define D7_CONTROL              0x0C
#define D8_CONTROL              0x0D
#define D9_CONTROL              0x0E

#define D1_PWM                  0x16     
#define D2_PWM                  0x17     
#define D3_PWM                  0x18     
#define D4_PWM                  0x19     
#define D5_PWM                  0x1A     
#define D6_PWM                  0x1B     
#define D7_PWM                  0x1C     
#define D8_PWM                  0x1D     
#define D9_PWM                  0x1E

#define D1_CURRENT_CONTROL      0x26
#define D2_CURRENT_CONTROL      0x27
#define D3_CURRENT_CONTROL      0x28
#define D4_CURRENT_CONTROL      0x29
#define D5_CURRENT_CONTROL      0x2A
#define D6_CURRENT_CONTROL      0x2B
#define D7_CURRENT_CONTROL      0x2C
#define D8_CURRENT_CONTROL      0x2D
#define D9_CURRENT_CONTROL      0x2E

#define MISC                    0x36
#define ENGINE1PC               0x37
#define ENGINE2PC               0x38
#define ENGINE3PC               0x39
#define STATUS_INTERRUPT        0x3A
#define INT/GPO                 0x3B
#define VARIABLE                0x3C
#define RESET                   0x3D
#define TEMP_ADC_CONTROL        0x3E
#define TEMPERATURE_READ        0x3F
#define TEMPERATURE_WRITE       0x40
#define LED_TEST_CONTROL        0x41
#define LED_TEST_ADC            0x42

#define ENGINE1_VARIABLE_A      0x45
#define ENGINE2_VARIABLE_A      0x46
#define ENGINE3_VARIABLE_A      0x47
#define MASTER_FADER_1          0x48
#define MASTER_FADER_2          0x49
#define MASTER_FADER_3          0x4A

#define ENG1_PROG_START_ADDR    0x4C
#define ENG2_PROG_START_ADDR    0x4D
#define ENG3_PROG_START_ADDR    0x4E
#define PROG_MEM_PAGE_SEL       0x4F

#define PROG_MEM_START          0x50 // 16-bit instructions, two registers per instruction, 96-instructions

#define ENG1_MAPPING_MSB        0x70
#define ENG1_MAPPING_LSB        0x71
#define ENG2_MAPPING_MSB        0x72
#define ENG2_MAPPING_LSB        0x73
#define ENG3_MAPPING_MSB        0x74
#define ENG3_MAPPING_LSB        0x75

#define GAIN_CHANGE_CTRL        0x76

#define LP55231_Address         0x32   // Address of the LP55231

const boolean invert = false; // set true if common anode, false if common cathode

uint8_t color = 0;        // a value from 0 to 255 representing the hue
uint8_t R, G, B;          // the Red Green and Blue color components
uint8_t brightness = 255; // 255 is maximum brightness

void setup()
{
  // Setup for Master mode, pins 18/19, external pullups, 400kHz for Teensy 3.1
  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_16_17, I2C_PULLUP_EXT, I2C_RATE_400);
  Serial.begin(38400);
  delay(5000);

  I2Cscan(); // should detect LP55231 at 32 

  // enable the chip
  writeByte(LP55231_Address, ENABLE_ENGINE_CNTRL1, 0x40);
  
// Enable auto increment (bit 6), auto charge pump (bits 4;3), and set the clock to internal mode bits 1:0
  writeByte(LP55231_Address, MISC, 0x40 | 0x18 | 0x03);
  

  // Set PWM output
   writeByte(LP55231_Address, D1_PWM, 0x40);
   writeByte(LP55231_Address, D2_PWM, 0x20);
   writeByte(LP55231_Address, D3_PWM, 0x80);
   writeByte(LP55231_Address, D4_PWM, 0x40);
   writeByte(LP55231_Address, D5_PWM, 0x20);
   writeByte(LP55231_Address, D6_PWM, 0x80);
   writeByte(LP55231_Address, D7_PWM, 0x40);
   writeByte(LP55231_Address, D8_PWM, 0x20);
   writeByte(LP55231_Address, D9_PWM, 0x80);

  // Set current output
   writeByte(LP55231_Address, D1_CURRENT_CONTROL, 0x0A);
   writeByte(LP55231_Address, D2_CURRENT_CONTROL, 0x09);
   writeByte(LP55231_Address, D3_CURRENT_CONTROL, 0x0B);
   writeByte(LP55231_Address, D4_CURRENT_CONTROL, 0x0A);
   writeByte(LP55231_Address, D5_CURRENT_CONTROL, 0x09);
   writeByte(LP55231_Address, D6_CURRENT_CONTROL, 0x0B);
   writeByte(LP55231_Address, D7_CURRENT_CONTROL, 0x0A);
   writeByte(LP55231_Address, D8_CURRENT_CONTROL, 0x09);
   writeByte(LP55231_Address, D9_CURRENT_CONTROL, 0x0B);

  // configure temperature sensor
  writeByte(LP55231_Address, TEMP_ADC_CONTROL, 0x06); // continuous conversion 

  // test of led function
  for (int ii = 0; ii < 8; ii++)
  {
  writeByte(LP55231_Address, OUTPUT_ON_OFF_CNTL_LSB, 1 << ii);
  delay(1000);
  writeByte(LP55231_Address, OUTPUT_ON_OFF_CNTL_LSB, 0x00);
  }
 
  writeByte(LP55231_Address, OUTPUT_ON_OFF_CNTL_MSB, 0x01);
  delay(1000);
  writeByte(LP55231_Address, OUTPUT_ON_OFF_CNTL_MSB, 0x00);
 
 // start with all leds off
  writeByte(LP55231_Address, OUTPUT_ON_OFF_CNTL_MSB, 0x00); 
  writeByte(LP55231_Address, OUTPUT_ON_OFF_CNTL_LSB, 0x00);
  
}


void loop()
{  

    checkTemp();

    writeByte(LP55231_Address, OUTPUT_ON_OFF_CNTL_LSB, 0x07);
    for (color = 0; color < 255; color++) { // Slew through the color spectrum

    hueToRGB(color, brightness);  // call function to convert hue to RGB

    // write the RGB values to the pins
    writeByte(LP55231_Address, D1_PWM, R);
    writeByte(LP55231_Address, D2_PWM, G);
    writeByte(LP55231_Address, D3_PWM, B);
  
    delay(100);
    }

    checkTemp();

    writeByte(LP55231_Address, OUTPUT_ON_OFF_CNTL_LSB, 0x38);
    for (color = 0; color < 255; color++) { // Slew through the color spectrum

    hueToRGB(color, brightness);  // call function to convert hue to RGB

    // write the RGB values to the pins
    writeByte(LP55231_Address, D4_PWM, R);
    writeByte(LP55231_Address, D5_PWM, G);
    writeByte(LP55231_Address, D6_PWM, B);
  
    delay(100);
    }

    checkTemp();

    writeByte(LP55231_Address, OUTPUT_ON_OFF_CNTL_LSB, 0xC0);
    writeByte(LP55231_Address, OUTPUT_ON_OFF_CNTL_MSB, 0x01);
    for (color = 0; color < 255; color++) { // Slew through the color spectrum

    hueToRGB(color, brightness);  // call function to convert hue to RGB

    // write the RGB values to the pins
    writeByte(LP55231_Address, D7_PWM, R);
    writeByte(LP55231_Address, D8_PWM, G);
    writeByte(LP55231_Address, D9_PWM, B);
  
    delay(100);
    }

    // turn off all leds
    writeByte(LP55231_Address, OUTPUT_ON_OFF_CNTL_LSB, 0x00);
    writeByte(LP55231_Address, OUTPUT_ON_OFF_CNTL_MSB, 0x00);
   
}


//===================================================================================================================
//====== Set of useful function to access acceleration. gyroscope, magnetometer, and temperature data
//===================================================================================================================

// Function to check temperature and turn off leds for 10 sec if they are too hot
void checkTemp()
{
  // read temperature
    int8_t temp = readByte(LP55231_Address, TEMPERATURE_READ);
    Serial.print("Temperature of LP55231 is "); Serial.print(temp); Serial.println(" degrees");
    if(temp > 38) {
      // if temperature too high, turn all leds off!
         writeByte(LP55231_Address, OUTPUT_ON_OFF_CNTL_MSB, 0x00); 
         writeByte(LP55231_Address, OUTPUT_ON_OFF_CNTL_LSB, 0x00);
         Serial.println("Toohot, shutting down for 10 seconds!");
         delay(10000);
         }
}
         
// simple function to scan for I2C devices on the bus
void I2Cscan() 
{
    // scan for i2c devices
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ ) 
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error==4) 
    {
      Serial.print("Unknow error at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
}


// Courtesy http://www.instructables.com/id/How-to-Use-an-RGB-LED/?ALLSTEPS
// function to convert a color to its Red, Green, and Blue components.

void hueToRGB(uint8_t hue, uint8_t brightness)
{
    uint16_t scaledHue = (hue * 6);
    uint8_t segment = scaledHue / 256; // segment 0 to 5 around the
                                            // color wheel
    uint16_t segmentOffset =
      scaledHue - (segment * 256); // position within the segment

    uint8_t complement = 0;
    uint16_t prev = (brightness * ( 255 -  segmentOffset)) / 256;
    uint16_t next = (brightness *  segmentOffset) / 256;

    if(invert)
    {
      brightness = 255 - brightness;
      complement = 255;
      prev = 255 - prev;
      next = 255 - next;
    }

    switch(segment ) {
    case 0:      // red
        R = brightness;
        G = next;
        B = complement;
    break;
    case 1:     // yellow
        R = prev;
        G = brightness;
        B = complement;
    break;
    case 2:     // green
        R = complement;
        G = brightness;
        B = next;
    break;
    case 3:    // cyan
        R = complement;
        G = prev;
        B = brightness;
    break;
    case 4:    // blue
        R = next;
        G = complement;
        B = brightness;
    break;
   case 5:      // magenta
    default:
        R = brightness;
        G = complement;
        B = prev;
    break;
    }
}



// I2C read/write functions for the MPU6500 and AK8963 sensors

        void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
	Wire.beginTransmission(address);  // Initialize the Tx buffer
	Wire.write(subAddress);           // Put slave register address in Tx buffer
	Wire.write(data);                 // Put data in Tx buffer
	Wire.endTransmission();           // Send the Tx buffer
}

        uint8_t readByte(uint8_t address, uint8_t subAddress)
{
	uint8_t data; // `data` will store the register data	 
	Wire.beginTransmission(address);         // Initialize the Tx buffer
	Wire.write(subAddress);	                 // Put slave register address in Tx buffer
	Wire.endTransmission(I2C_NOSTOP);        // Send the Tx buffer, but send a restart to keep connection alive
//	Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
//	Wire.requestFrom(address, 1);  // Read one byte from slave register address 
	Wire.requestFrom(address, (size_t) 1);   // Read one byte from slave register address 
	data = Wire.read();                      // Fill Rx buffer with result
	return data;                             // Return data read from slave register
}

        void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{  
	Wire.beginTransmission(address);   // Initialize the Tx buffer
	Wire.write(subAddress);            // Put slave register address in Tx buffer
	Wire.endTransmission(I2C_NOSTOP);  // Send the Tx buffer, but send a restart to keep connection alive
//	Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
	uint8_t i = 0;
//        Wire.requestFrom(address, count);  // Read bytes from slave register address 
        Wire.requestFrom(address, (size_t) count);  // Read bytes from slave register address 
	while (Wire.available()) {
        dest[i++] = Wire.read(); }         // Put read results in the Rx buffer
}
