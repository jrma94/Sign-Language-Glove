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
int x,y,z;
int xg,yg,zg;

// the setup routine runs once when you press reset:
const int buttonPin = PUSH2;     // the number of the pushbutton pin
const int ledPin =  RED_LED;      // the number of the LED pin
const int sampleSize = 20;    // read every 20th sample
const int QUAN = 25;          // quantization level
//const int PERIOD = 20;        //period for moving average

// variables will change:
int buttonState = 0;         // variable for reading the pushbutton status
int r;
// Declare variables
int sensor1 = 0,sensor2 = 0,sensor3 = 0;
int sensor4 = 0,sensor5= 0;
int thumb_cont = 0, index_cont1 = 0, index_cont2 = 0;
int contact1 = 0, contact2 = 0, contact3 = 0;
// pin numbers for the three contacts
const int cont1 = 40, cont2 = 39, cont3 = 38;
int QDATA1[10],QDATA2[10],QDATA3[10],QDATA4[10],QDATA5[10]; // holds quantized data
float QDER1[10], QDER2[10], QDER3[10], QDER4[10], QDER5[10]; // holds quantized derivative
float slope1[20], slope2[20], slope3[20], slope4[20], slope5[20];
float T_slope, I_slope, M_slope, R_slope, P_slope;

//float MAccel1[20], MAccel2[20], MAccel3[20];                  //holds the moving average of the accelerometer
//float DMAccel1[20],DMAccel2[20],DMAccel3[20];               // hold the derivative of the moving average
//float MATotal1[1]={0}, MATotal2[1]={0}, MATotal3[1]={0};          // holds the moving average total
char Letter; // holds character
char oldletter = ',';

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
  
  Serial.println(F("Sign Language Glove Translator"));
  Serial.println(F("To go to the next line press button 2 on the microcontroller"));
  Serial.println(F("Begin:"));
}

// the loop routine runs over and over again forever:
void loop() {
  
  // read the state of the pushbutton value:
  buttonState = digitalRead(buttonPin);

  // check if the pushbutton is pressed.
  // if it is, the buttonState is HIGH:
  if(buttonState == LOW){
        Serial.println(':');
        Letter = ',';
        oldletter = '.';
  }    
    // turn LED on:    
    digitalWrite(ledPin, HIGH);
    // read samples
        for(int i = 0; i<sampleSize; i++){  // read every 20th sample
        sensor1 = round(analogRead(23)/QUAN)*QUAN;
        sensor2 = round(analogRead(24)/QUAN)*QUAN;
        sensor3 = round(analogRead(25)/QUAN)*QUAN;
        sensor4 = round(analogRead(26)/QUAN)*QUAN;
        sensor5 = round(analogRead(27)/QUAN)*QUAN;
       
        
        slope1[i] = sensor1;
        slope2[i] = sensor2;
        slope3[i] = sensor3;
        slope4[i] = sensor4;
        slope5[i] = sensor5;
        } 
        
        T_slope = (slope1[19]-slope1[0])/(20-1);
        I_slope = (slope2[19]-slope2[0])/(20-1);
        M_slope = (slope3[19]-slope3[0])/(20-1);
        R_slope = (slope4[19]-slope4[0])/(20-1);
        P_slope = (slope5[19]-slope5[0])/(20-1);
        
 
     //Reading 6 bytes of data starting at register DATAX0 will retrieve the x,y and z acceleration values from the ADXL345.
     //The results of the read operation will get stored to the values[] buffer.
     readRegister(DATAX0, 6, values);

     //The ADXL345 gives 10-bit acceleration values, but they are stored as bytes (8-bits). To get the full value, two bytes must be combined for each axis.
     //The X value is stored in values[0] and values[1].
     //***Since the current x value, as written below, outputs it in the negative direction when starting from a neutral position, flip it and see if this gives a positive result!!***
     x = -1*(((int)values[1]<<8)|(int)values[0]);
     
     //The Y value is stored in values[2] and values[3].
     y = -1*(((int)values[3]<<8)|(int)values[2]);
     
     //The Z value is stored in values[4] and values[5].
     z = -1*(((int)values[5]<<8)|(int)values[4]);
     
     //Read contact states
     contact1 = digitalRead(cont1); // thumb contact connected to pin 40
     if(contact1 == HIGH){
       thumb_cont = 1;
     }
     else{
       thumb_cont = 0;
     }
     contact2 = digitalRead(cont2); // thumb contact connected to pin 39
     if(contact2 == HIGH){
       index_cont1 = 1;
     }
     else{
       index_cont1 = 0;
     }
     contact3 = digitalRead(cont3); // thumb contact connected to pin 38
     if(contact3 == HIGH){
       index_cont2 = 1;
     }
     else{
       index_cont2 = 0;
     }
     
     //Print all the flex sensor values, accelerometer values, and contact state values.
     /*
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
     Serial.print(z, DEC);
     Serial.print(',');
     Serial.print(thumb_cont);
     Serial.print(',');
     Serial.print(index_cont1);
     Serial.print(',');
     Serial.println(index_cont2);
     */

      if((-1.00 <= T_slope <= 1.0)&&(-1.00 <= I_slope <= 1.0)&&(-1.00 <= M_slope <= 1.0)&&(-1.00 <= R_slope <= 1.0)&&(-1.00 <= P_slope <= 1.0)){
      //Thumb, Index, Middle, Ring, Pinky, Contacts, Accel
      Letter = lettersProfile_1(sensor1, sensor2, sensor3, sensor4, sensor5, contact1, contact2, contact3, x, y, z, oldletter);
      
      if(Letter != oldletter){
               Serial.print(Letter);
               oldletter = Letter;
          }
          
      }   
else {
    // turn LED off:
digitalWrite(ledPin, LOW); 
   }

delay(300);
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

char lettersProfile_1(int thumb, int index, int middle, int ring, int pinky, int c1, int c2, int c3, int x, int y, int z, char old_letter) {
  char letter;
  
  // Letter profile for Carlos
 //Letter a
         if (((400 <= thumb) && (thumb<= 475)) && ((350 <= index) && (index<= 400)) && ((300 <= middle) && (middle<= 350)) && 
            ((200 <= ring) && (ring<= 250)) && ((250 <= pinky) && (pinky<= 275))&& 
            (c1 == LOW)&& (c2 == LOW) && (c3 == HIGH))
         {
             letter = 'a';
           }
           
  //Letter a (Robert)
        else if (((450 <= thumb) && (thumb <= 525)) && ((375 <= index) && (index<= 400)) && ((350 <= middle) && (middle<= 375)) && 
            ((250 <= ring) && (ring<= 275)) && ((275 <= pinky) && (pinky<= 300))&& 
            (c1 == LOW)&& (c2 == LOW) && (c3 == HIGH))
         {
             letter = 'a';
           }
          
    //letter b
        else if (((300 <= thumb) && (thumb<= 350)) && ((525 <= index) && (index<= 550)) && ((500 <= middle) && (middle<= 550)) && 
            ((450 <= ring) && (ring<= 500)) && ((500 <= pinky) && (pinky<= 525))&& 
            (c1 == LOW) && (c2 == LOW) && (c3 == HIGH))
        {
             letter = 'b';
           }
    //letter c
        else if (((375 <= thumb) && (thumb<= 425)) && ((475 <= index) && (index<= 500)) && ((400 <= middle) && (middle<= 450)) && 
            ((325  <= ring) && (ring<= 400)) && ((425 <= pinky) && (pinky<= 450))&& 
            (c1 == LOW)&& (c2 == LOW) && (c3 == HIGH))
        {
             letter = 'c';
             }   
    //letter d
        else if (((325 <= thumb) && (thumb<= 375)) && ((500 <= index) && (index<= 550)) && ((375 <= middle) && (middle<= 400)) && 
            ((275 <= ring) && (ring<= 300)) && ((325 <= pinky) && (pinky<= 375)) && 
            (c1 == HIGH)&& (c2 == LOW)&& (c3 == LOW))
        {
            letter = 'd';
             }   
    //letter e
        else if (((300 <= thumb) && (thumb<= 325)) && ((400 <= index) && (index<= 425)) && ((350 <= middle) && (middle<= 375)) && 
            ((275 <= ring) && (ring<= 300)) && ((300 <= pinky) && (pinky<= 325))&& 
            (c1 == LOW)&& (c2 == LOW))
        {
            letter = 'e';
             } 

    //letter e (Robert)
        else if (((250 <= thumb) && (thumb<= 275)) && ((400 <= index) && (index<= 425)) && ((350 <= middle) && (middle<= 375)) && 
            ((250 <= ring) && (ring<= 275)) && ((300 <= pinky) && (pinky<= 325))&& 
            (c1 == LOW)&& (c2 == LOW))
        {
            letter = 'e';
             }
             
    //letter f
        else if (((375 <= thumb) && (thumb<= 400)) && ((400 <= index) && (index<= 425)) && ((500 <= middle) && (middle<= 525)) && 
            ((425 <= ring) && (ring<= 475)) && ((475 <= pinky) && (pinky<= 525))  && 
            (c1 == LOW) && (c2 == LOW) && (c3 == LOW))
        {
            letter = 'f';
             }
    //letter g
        else if (((425 <= thumb) && (thumb<= 475)) && ((475 <= index) && (index<= 500)) && ((325 <= middle) && (middle<= 375)) && 
            ((225 <= ring) && (ring<= 275)) && ((275 <= pinky) && (pinky<= 300)) && 
            (c1 == LOW) && (c2 == LOW) && (c3 == LOW)&& (x < 20)&& (y > 20)&&(z < 20))
        {
            letter = 'g';
             }
    //letter h
        else if (((300 <= thumb) && (thumb<= 375)) && ((500 <= index) && (index<= 550)) && ((450 <= middle) && (middle<= 500)) && 
            ((225 <= ring) && (ring<= 250)) && ((275 <= pinky) && (pinky<= 350))&& 
            (c1 == LOW)&& (c2 == LOW) && (c3 == HIGH)&& (x < 25)&& (y > 20)&&(z < 25))
        {
               letter = 'h';
             }
    //letter i
        else if (((300 <= thumb) && (thumb<= 350)) && ((375 <= index) && (index<= 400)) && ((350 <= middle) && (middle<= 375)) && 
            ((250 <= ring) && (ring<= 275)) && ((450 <= pinky) && (pinky<= 500)) && 
            (c1 == HIGH)&& (c2 == HIGH) && (c3 == HIGH)&& (x > 20)&& (y < 25)&&(z < 25))
        {
            letter = 'i';
             }
    //letter j
         else if (((300 <= thumb) && (thumb<= 350)) && ((375 <= index) && (index<= 400)) && ((350 <= middle) && (middle<= 375)) && 
             ((250 <= ring) && (ring<= 275)) && ((450 <= pinky) && (pinky<= 500)) && 
             (c1 == HIGH) && (c2 == HIGH) && (c3 == HIGH)&& (x < 10)&& (y > 20)&&(z < 25))
         {
             letter = 'j';
             }
    //letter k
        else if (((475 <= thumb) && (thumb<= 525)) && ((500 <= index) && (index<= 525)) && ((425 <= middle) && (middle<= 450)) && 
            ((250 <= ring) && (ring<= 275)) && ((300 <= pinky) && (pinky<= 325)) && 
            (c1 == LOW) && (c2 == LOW) && (c3 == LOW)&& (x > 20)&& (y < 25)&&(z < 25))
        {
            letter = 'k';
             }
    //letter l
        else if (((550 <= thumb) && (thumb<= 575)) && ((525 <= index) && (index<= 550)) && ((325 <= middle) && (middle<= 350)) && 
            ((225 <= ring) && (ring<= 250)) && ((275 <= pinky) && (pinky<= 325)) && 
            (c1 == LOW) && (c2 == LOW) && (c3 == LOW))
        {
            letter = 'l';
             }
    //letter m
         else if (((325 <= thumb) && (thumb<= 375)) && ((375 <= index) && (index<= 400)) && ((325 <= middle) && (middle<= 375)) && 
             ((250 <= ring) && (ring<= 325)) && ((350 <= pinky) && (pinky<= 400))&& 
             (c1 == LOW)&& (c2 == LOW) && (c3 == HIGH))
         {
             letter = 'm';
             }
    //letter n
        else if (((375 <= thumb) && (thumb<= 425)) && ((375 <= index) && (index<= 400)) && ((375 <= middle) && (middle<= 400)) && 
            ((250 <= ring) && (ring<= 300)) && ((275 <= pinky) && (pinky<= 350))&& 
            (c1 == LOW)&& (c2 == LOW) && (c3 == HIGH))
        {
            letter = 'n';
             }
    //letter o
        else if (((325 <= thumb) && (thumb<= 350)) && ((400 <= index) && (index<= 425)) && ((375 <= middle) && (middle<= 425)) && 
            ((300 <= ring) && (ring<= 325)) && ((350 <= pinky) && (pinky<= 375)) && 
            (c1 == HIGH)&& (c2 == LOW) && (c3 == HIGH))
        {
            letter = 'o';
             }

//letter o (Robert)
        else if (((325 <= thumb) && (thumb<= 350)) && ((400 <= index) && (index<= 425)) && ((375 <= middle) && (middle<= 425)) && 
            ((275 <= ring) && (ring<= 300)) && ((325 <= pinky) && (pinky<= 350)) && 
            (c1 == HIGH)&& (c2 == LOW) && (c3 == HIGH))
        {
            letter = 'o';
             } 
             
    //letter p
        else if (((425 <= thumb) && (thumb<= 450)) && ((500 <= index) && (index<= 525)) && ((425 <= middle) && (middle<= 450)) && 
            ((250 <= ring) && (ring<= 275)) && ((325 <= pinky) && (pinky<= 350)) && 
            (c1 == LOW) && (c2 == LOW) && (c3 == LOW)&& (x < 0)&& (y >0)&&(z < 0))
        {
            letter = 'p';
             }
            
//letter p (Robert)
        else if (((400 <= thumb) && (thumb<= 450)) && ((475 <= index) && (index<= 500)) && ((375 <= middle) && (middle<= 400)) && 
            ((250 <= ring) && (ring<= 275)) && ((275 <= pinky) && (pinky<= 325)) && 
            (c1 == LOW) && (c2 == LOW) && (c3 == LOW)&& (x < 0)&& (y >0)&&(z < 0))
        {
            letter = 'p';
             }
             
//letter q (Robert)
        else if (((450 <= thumb) && (thumb<= 500)) && ((475 <= index) && (index<= 500)) && ((350 <= middle) && (middle<= 375)) && 
            ((225 <= ring) && (ring<= 250)) && ((275 <= pinky) && (pinky<= 300)) && 
            (c2 == LOW) && (c3 == LOW)&& (x < 0)&& (y > 0)&&(z < 0))
        {
            letter = 'q';
             }
     //letter r
        else if (((350 <= thumb) && (thumb<= 375)) && ((500 <= index) && (index<= 525)) && ((450 <= middle) && (middle<= 475)) && 
            ((275 <= ring) && (ring<= 300)) && ((325 <= pinky) && (pinky<= 350))&& 
            (c1 == LOW) && (c2 == HIGH)&& (c3 == LOW))
        {
            letter = 'r';
             }
             
//letter r (Robert)
        else if (((300 <= thumb) && (thumb<= 350)) && ((500 <= index) && (index<= 525)) && ((450 <= middle) && (middle<= 500)) && 
            ((250 <= ring) && (ring<= 300)) && ((300 <= pinky) && (pinky<= 350))&& 
            (c1 == LOW) && (c2 == HIGH)&& (c3 == LOW))
        {
            letter = 'r';
             }
     
//letter s (Robert)
        else if (((325 <= thumb) && (thumb<= 350)) && ((350 <= index) && (index<= 400)) && ((325 <= middle) && (middle<= 350)) && 
            ((225 <= ring) && (ring<= 250)) && ((275 <= pinky) && (pinky<= 300)) && 
            (c2 == LOW) && (c3 == HIGH))
        {
            letter = 's';
             }
     //letter t
        else if (((400 <= thumb) && (thumb<= 450)) && ((400 <= index) && (index<= 450)) && ((350 <= middle) && (middle<= 400)) && 
            ((250 <= ring) && (ring<= 275)) && ((300 <= pinky) && (pinky<= 350)) && 
            (c1 == LOW) && (c2 == LOW) && (c3 == LOW))
        {
            letter = 't';
             }
     //letter u
          else if (((350 <= thumb) && (thumb<= 375)) && ((500 <= index) && (index<= 550)) && ((475 <= middle) && (middle<= 525)) && 
              ((275 <= ring) && (ring<= 300)) && ((350 <= pinky) && (pinky<= 375))&& 
              (c1 == LOW)&& (c2 == LOW) && (c3 == HIGH)&&(x > 25)&& (y < 25)&&(z < 25))
          {
              letter = 'u';
             }
             
//letter u(Robert)
          else if (((325 <= thumb) && (thumb<= 350)) && ((500 <= index) && (index<= 550)) && ((475 <= middle) && (middle<= 525)) && 
              ((225 <= ring) && (ring<= 275)) && ((300 <= pinky) && (pinky<= 350))&& 
              (c1 == LOW)&& (c2 == LOW) && (c3 == HIGH)&&(x > 20)&& (y < 20)&&(z < 20))
          {
              letter = 'u';
             }
     //letter v
        else if (((325 <= thumb) && (thumb<= 400)) && ((525 <= index) && (index<= 575)) && ((500 <= middle) && (middle<= 525)) && 
            ((250 <= ring) && (ring<= 300)) && ((325 <= pinky) && (pinky<= 350)) && 
            (c1 == LOW) && (c2 == LOW) && (c3 == LOW))
        {
            letter = 'v';
             }
     //letter w
         else if ((((300 <= thumb) && (thumb<= 375)) && ((500 <= index) && (index<= 550)) && ((500 <= middle) && (middle<= 550)) && 
             ((425 <= ring) && (ring)<= 475)) && ((275 <= pinky) && (pinky<= 350)) && 
             (c1 == LOW) && (c2 == LOW) && (c3 == LOW))
         {
             letter = 'w';
             }
     //letter x
       else if (((300 <= thumb) && (thumb<= 350)) && ((425 <= index) && (index<= 475)) && ((325 <= middle) && (middle<= 350)) && 
           ((225 <= ring) && (ring<= 275)) && ((275 <= pinky) && (pinky<= 300)) && 
           (c1 == HIGH) && (c3 == LOW) && (c3 == LOW))
           {
           letter = 'x';
             }
     //letter y
         else if (((550 <= thumb) && (thumb<= 575)) && ((375 <= index) && (index<= 400)) && ((350 <= middle) && (middle<= 375)) && 
             ((250 <= ring) && (ring<= 275)) && ((475 <= pinky) && (pinky<= 525))&& 
             (c1 == LOW) && (c2 == LOW) && (c3 == HIGH))
         {
             letter = 'y';
             }
     //letter z
        else if (((325 <= thumb) && (thumb<= 375)) && ((500 <= index) && (index<= 525)) && ((325 <= middle) && (middle<= 350)) && 
            ((250 <= ring) && (ring<= 275)) && ((275 <= pinky) && (pinky<= 300)) && 
            (c2 == LOW) && (c3 == LOW)&&(x < 12)&& (y > 20)&&(z < 0)) 
         {
            letter = 'z';
             } 
         else {
          letter = old_letter; 
         }
  return letter;
}
