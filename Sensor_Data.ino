//Add the SPI library so we can communicate with the ADXL345 sensor
#include <SPI.h>

//Assign the Chip Select signal to pin 8.
int CS=8;

//This is a list of some of the registers available on the ADXL345.
//To learn more about these and the rest of the registers on the ADXL345, read the datasheet!
char POWER_CTL = 0x2D;	//Power Control Register
char DATA_FORMAT = 0x31;
char DATAX0 = 0x32;	//X-Axis Data 0
char DATAX1 = 0x33;	//X-Axis Data 1
char DATAY0 = 0x34;	//Y-Axis Data 0
char DATAY1 = 0x35;	//Y-Axis Data 1
char DATAZ0 = 0x36;	//Z-Axis Data 0
char DATAZ1 = 0x37;	//Z-Axis Data 1

//This buffer will hold values read from the ADXL345 registers.
unsigned char values[10];
//These variables will be used to hold the x,y and z axis accelerometer values.
int16_t x,y,z;
int16_t xg,yg,zg;

// the setup routine runs once when you press reset:
const int buttonPin = PUSH2;     // the number of the pushbutton pin
const int ledPin =  RED_LED;      // the number of the LED pin
const int sampleSize = 10;    // read every 20th sample
const int QUAN = 10;

// variables will change:
int buttonState = 1;         // variable for reading the pushbutton status
bool buttonToggle = false;  // variable for saving the toggle state of the code (running when true)

// Declare variables
int sensor1 = 0,sensor2 = 0,sensor3 = 0;
int sensor4 = 0,sensor5= 0;
int thumb_cont = 0, index_cont1 = 0, index_cont2 = 0;
int contact1 = 0, contact2 = 0, contact3 = 0;
// pin numbers for the three contacts
const int cont1 = 40, cont2 = 39, cont3 = 38;
int QDATA1[20],QDATA2[20],QDATA3[20],QDATA4[20],QDATA5[20]; // holds quantized data

char Letter[21]; // holds character

void setup() {
  
  Serial.begin(9600); // msp430g2231 must use 4800
  analogReadResolution(10);  // Use 10 bit resolution instead of the native 12 bit
  
  //Initiate an SPI communication instance.
  SPI.begin();
  //Configure the SPI connection for the ADXL345.
  SPI.setDataMode(SPI_MODE3);
  
  pinMode(ledPin, OUTPUT);
  // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT_PULLUP);
  
  //Set up the Chip Select pin to be an output from the Arduino.
  pinMode(CS, OUTPUT);
  //Before communication starts, the Chip Select pin needs to be set high.
  digitalWrite(CS, HIGH);
  
  //Put the ADXL345 into +/- 8G range by writing the value 0x01 to the DATA_FORMAT register.
  writeRegister(DATA_FORMAT, 0x03);
  //Put the ADXL345 into Measurement Mode by writing 0x08 to the POWER_CTL register.
  writeRegister(POWER_CTL, 0x08);  //Measurement mode
  // pin modes
  pinMode (cont1, INPUT);
  pinMode (cont2, INPUT);
  pinMode (cont3, INPUT);
}

// the loop routine runs over and over again forever:
void loop() {
  // read the state of the pushbutton value:
  buttonState = digitalRead(buttonPin);

  // check if the pushbutton is pressed.
  // if it is, the buttonState is HIGH:
  // When button state is HIGH, change value of buttonToggle
  if(buttonState == LOW){
    if(buttonToggle){
      buttonToggle = false;
    }else{
      buttonToggle = true;
    }

    //wait for button to be depressed to start main loop
    while(buttonState == LOW){
      buttonState = digitalRead(buttonPin);
    }
  }
  
  if(buttonToggle){    
    // turn LED on:    
    digitalWrite(ledPin, HIGH);
    // read samples
    //for(int j = 0; j < 20; j++) {
      for(int i = 0; i<sampleSize; i++){  // read every 20th sample
        sensor1 = round(analogRead(23)/QUAN)*QUAN;
        sensor2 = round(analogRead(24)/QUAN)*QUAN;
        sensor3 = round(analogRead(25)/QUAN)*QUAN;
        sensor4 = round(analogRead(26)/QUAN)*QUAN;
        sensor5 = round(analogRead(27)/QUAN)*QUAN;
        } 
 
     //Reading 6 bytes of data starting at register DATAX0 will retrieve the x,y and z acceleration values from the ADXL345.
     //The results of the read operation will get stored to the values[] buffer.
     readRegister(DATAX0, 6, values);

     //The ADXL345 gives 10-bit acceleration values, but they are stored as bytes (8-bits). To get the full value, two bytes must be combined for each axis.
     //The X value is stored in values[0] and values[1].
     //***Since the current x value, as written below, outputs it in the negative direction when starting from a neutral position, flip it and see if this gives a positive result!!***
     x = -1*(((int16_t)values[1]<<8)|(int16_t)values[0]);
     //The Y value is stored in values[2] and values[3].
     y = -1*(((int16_t)values[3]<<8)|(int16_t)values[2]);
     //The Z value is stored in values[4] and values[5].
     z = -1*(((int16_t)values[5]<<8)|(int16_t)values[4]);
     
     /*
     //Read contact states
     contact1 = digitalRead(cont1); // thumb contact connected to pin 40
     if(contact1 == HIGH){
       thumb_cont = 1;
     }
     else{
       thumb_cont = 0;
     }
     contact2 = digitalRead(cont2); // thumb contact connected to pin 40
     if(contact2 == HIGH){
       index_cont1 = 1;
     }
     else{
       index_cont1 = 0;
     }
     contact3 = digitalRead(cont3); // thumb contact connected to pin 40
     if(contact3 == HIGH){
       index_cont2 = 1;
     }
     else{
       index_cont2 = 0;
     }*/
     
     //Print all the flex sensor values, accelerometer values, and contact state values.
     Serial.print(sensor1);
     Serial.print(',');
     Serial.print(sensor2);
     Serial.print(',');
     Serial.print(sensor3);
     Serial.print(',');
     Serial.print(sensor4);
     Serial.print(',');
     Serial.print(sensor5);
     Serial.print(',');
     Serial.print(x, DEC);
     Serial.print(',');
     Serial.print(y, DEC);
     Serial.print(',');
     Serial.println(z, DEC);
     //Serial.print(',');
     //Serial.print(thumb_cont);
     //Serial.print(',');
     //Serial.print(index_cont1);
     //Serial.print(',');
     //Serial.println(index_cont2);
     
       // hold the quantized data
       //QDATA1[j] = sensor1;
       //QDATA2[j] = sensor2;
       //QDATA3[j] = sensor3;
       //QDATA4[j] = sensor4;
       //QDATA5[j] = sensor5;
     //}
     
     
    } else {
      digitalWrite(ledPin, LOW);
    }
}

//This function will write a value to a register on the ADXL345.
//Parameters:
//  char registerAddress - The register to write a value to
//  char value - The value to be written to the specified register.
void writeRegister(char registerAddress, char value){
  //Set Chip Select pin low to signal the beginning of an SPI packet.
  digitalWrite(CS, LOW);
  //Transfer the register address over SPI.
  SPI.transfer(registerAddress);
  //Transfer the desired register value over SPI.
  SPI.transfer(value);
  //Set the Chip Select pin high to signal the end of an SPI packet.
  digitalWrite(CS, HIGH);
}

//This function will read a certain number of registers starting from a specified address and store their values in a buffer.
//Parameters:
//  char registerAddress - The register addresse to start the read sequence from.
//  int numBytes - The number of registers that should be read.
//  char * values - A pointer to a buffer where the results of the operation should be stored.
void readRegister(char registerAddress, int numBytes, unsigned char * values){
  //Since we're performing a read operation, the most significant bit of the register address should be set.
  char address = 0x80 | registerAddress;
  //If we're doing a multi-byte read, bit 6 needs to be set as well.
  if(numBytes > 1)address = address | 0x40;
  
  //Set the Chip select pin low to start an SPI packet.
  digitalWrite(CS, LOW);
  //Transfer the starting register address that needs to be read.
  SPI.transfer(address);
  //Continue to read registers until we've read the number specified, storing the results to the input buffer.
  for(int i=0; i<numBytes; i++){
    values[i] = SPI.transfer(0x00);
  }
  //Set the Chips Select pin high to end the SPI packet.
  digitalWrite(CS, HIGH);
}

//This function will read the state of the contact and return its value.
int readContact(const int pin){
  int contact;
  int state;
  contact = digitalRead(pin);
  if(contact == LOW){
    state = 1;
    return state;
  }
  else{
   state = 0;
   return state; 
  }
 
}