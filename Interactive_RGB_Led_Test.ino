//************************************************************************************************************//
  
  // The 8x8 Interactive RGB LED Table

//************************************************************************************************************//

#include <SPI.h>// SPI Library used to clock data out to the shift registers

#define latch_pin 4// Defines actual BIT of PortD for latch - is Arduino UNO pin 2, MEGA pin 4
#define blank_pin 5// Defines actual BIT of PortD for blank - is Arduino UNO pin 3, MEGA pin 5
#define data_pin 51// used by SPI, must be pin MOSI 11 on Arduino UNO, 51 on MEGA
#define clock_pin 52// used by SPI, must be 13 SCK 13 on Arduino UNO, 52 on MEGA

//***************************************************Layer*********************************************************//
#define layer1    26 // bottom layer
#define layer2    27
#define layer3    28
#define layer4    29
#define layer5    30
#define layer6    31
#define layer7    32
#define layer8    33 // top layer


//*************************************************Phototransistor******************************************//

#define sense_select1   36    //8
#define sense_select2   38   //9
#define sense_select3   40  //10
#define ir_array_enable 42  //11     

#define LimitSense 150
#define fadetime   1


volatile int ir_sense_data[64];
volatile byte ir_group = 0;                    // to track current IR group
volatile byte ir_group_adder = 0;              // needed in interrupt to set appropriate IR node voltages read from array

unsigned long samplingtime = 0;

//************************************************************************************************************//

int layerArray[8] = {layer1, layer2, layer3, layer4, layer5, layer6, layer7, layer8};
int lastAnode;


byte red[4][32];
byte blue[4][32];
byte green[4][32];

//*********** Defining the Matrix *************

#define BAM_RESOLUTION 4    // EG 4 bit colour = 15 variation of R, G & B (4096 colours)
const byte Size_Y = 16;//Number of Layers Y axis (levels/Layers)
const byte Size_X = 16; //Number of LEDs X axis (Left to right across front)

int level=0;//keeps track of which level we are shifting data to
int anodeLevel=0;//this increments through the anode levels
int BAM_Bit, BAM_Counter=0; // Bit Angle Modulation variables to keep track of things

//****setup****setup****setup****setup****setup****setup****setup****setup****setup****setup****setup****setup****setup
void setup(){

SPI.setBitOrder(MSBFIRST);//Most Significant Bit First
SPI.setDataMode(SPI_MODE0);// Mode 0 Rising edge of data, keep clock low
SPI.setClockDivider(SPI_CLOCK_DIV2);//Run the data in at 16MHz/2 - 8MHz

noInterrupts();// kill interrupts until everybody is set up

//We use Timer 1 to refresh the cube
TCCR1A = B00000000;//Register A all 0's since we're not toggling any pins
TCCR1B = B00001011;//bit 3 set to place in CTC mode, will call an interrupt on a counter match
//bits 0 and 1 are set to divide the clock by 64, so 16MHz/64=250kHz
TIMSK1 = B00000010;//bit 1 set to call the interrupt on an OCR1A match
OCR1A=40; 

//finally set up the Outputs
// pinMode(latch_pin, OUTPUT);//Latch
pinMode (2, OUTPUT); // turn off PWM and set PortD bit 4 as output
pinMode (3, OUTPUT); // turn off PWM and set PortD bit 5 as output
pinMode(data_pin, OUTPUT);//MOSI DATA
pinMode(clock_pin, OUTPUT);//SPI Clock
//pinMode(blank_pin, OUTPUT);//Output Enable  important to do this last, so LEDs do not flash on boot up

//*** Here layer pins are set as outputs
pinMode(layer1, OUTPUT);
pinMode(layer2, OUTPUT);
pinMode(layer3, OUTPUT);
pinMode(layer4, OUTPUT);
pinMode(layer5, OUTPUT);
pinMode(layer6, OUTPUT);
pinMode(layer7, OUTPUT);
pinMode(layer8, OUTPUT);

// set pin mode for ir array enable
pinMode(ir_array_enable, OUTPUT);
digitalWrite(ir_array_enable, LOW);

//pin modes for IR sense muxes
//analogReadResolution(8);
pinMode(sense_select1, OUTPUT);
pinMode(sense_select2, OUTPUT);
pinMode(sense_select3, OUTPUT);

SPI.begin();//start up the SPI library
interrupts();//let the show begin, this lets the multiplexing start
//Serial.begin(9600); // sets the serial port to 9600


}//***end setup***end setup***end setup***end setup***end setup***end setup***end setup***end setup***end setup***end setup


void LED(int CX, int CY, int CR, int CG, int CB) { 

  CX = constrain(CX, 0, Size_X - 1);//Matrix X axis  
  CY = constrain(CY, 0, Size_Y - 1);//Matrix Y axis
  
  CR = constrain(CR, 0, (1 << BAM_RESOLUTION) - 1); //Red
  CG = constrain(CG, 0, (1 << BAM_RESOLUTION) - 1); //Green
  CB = constrain(CB, 0, (1 << BAM_RESOLUTION) - 1); //Blue

  int WhichByte = int(CY*2+CX/8);
  int WhichBit = CX%8;
  
  for (byte I = 0; I < BAM_RESOLUTION; I++) {
    //*** RED ***
    bitWrite(red[I][WhichByte], WhichBit, bitRead(CR, I));

    //*** GREEN ***
    bitWrite(green[I][WhichByte], WhichBit, bitRead(CG, I));

    //*** BLUE ***
    bitWrite(blue[I][WhichByte], WhichBit, bitRead(CB, I));
  }

}//****LED ROUTINE END****


ISR(TIMER1_COMPA_vect){//***MultiPlex BAM***MultiPlex BAM***MultiPlex BAM***MultiPlex BAM***MultiPlex BAM***MultiPlex BAM***MultiPlex BAM

if(BAM_Counter==8)
BAM_Bit++;
else
if(BAM_Counter==24)
BAM_Bit++;
else
if(BAM_Counter==56)
BAM_Bit++;

BAM_Counter++;

switch (BAM_Bit){
case 0:

      //Red
        myTransfer(red[0][level+16]);
        myTransfer(red[0][level+17]);        
        myTransfer(red[0][level]);
        myTransfer(red[0][level+1]);

              
      //Green
        myTransfer(green[0][level+16]);
        myTransfer(green[0][level+17]);      
        myTransfer(green[0][level]);
        myTransfer(green[0][level+1]);
        
        
      //Blue
        myTransfer(blue[0][level+16]);
        myTransfer(blue[0][level+17]);
        myTransfer(blue[0][level]);
        myTransfer(blue[0][level+1]);
           
      break;
    case 1:
       
      //Red
        myTransfer(red[1][level+16]);
        myTransfer(red[1][level+17]); 
        myTransfer(red[1][level]);
        myTransfer(red[1][level+1]);
       
      //Green
        myTransfer(green[1][level+16]);
        myTransfer(green[1][level+17]);  
        myTransfer(green[1][level]);
        myTransfer(green[1][level+1]);
       
      //Blue
        myTransfer(blue[1][level+16]);
        myTransfer(blue[1][level+17]);         
        myTransfer(blue[1][level]);
        myTransfer(blue[1][level+1]);
       
      break;
    case 2:
      
      //Red
        myTransfer(red[2][level+16]);
        myTransfer(red[2][level+17]);       
        myTransfer(red[2][level]);
        myTransfer(red[2][level+1]);
                
       //Green
        myTransfer(green[2][level+16]);
        myTransfer(green[2][level+17]);
        myTransfer(green[2][level]);
        myTransfer(green[2][level+1]);
         
      //Blue
        myTransfer(blue[2][level+16]);
        myTransfer(blue[2][level+17]);
        myTransfer(blue[2][level]);
        myTransfer(blue[2][level+1]);
         
      break;
    case 3:
      //Red
        myTransfer(red[3][level+16]);
        myTransfer(red[3][level+17]);
        myTransfer(red[3][level]);
        myTransfer(red[3][level+1]);

      //Green
        myTransfer(green[3][level+16]);
        myTransfer(green[3][level+17]);
        myTransfer(green[3][level]);
        myTransfer(green[3][level+1]);

      //Blue
        
        myTransfer(blue[3][level+16]);
        myTransfer(blue[3][level+17]);
        myTransfer(blue[3][level]);
        myTransfer(blue[3][level+1]);

  if(BAM_Counter==120){
  BAM_Counter=0;
  BAM_Bit=0;
  }
  break;
}

    lastAnode = (anodeLevel-1);
    if (anodeLevel == 0) { lastAnode = 7; } // if we are at the bottom, the last layer was the top
    digitalWrite(layerArray[lastAnode], LOW); // turn off the previous layer
    digitalWrite(layerArray[anodeLevel], HIGH); // turn on the current layer



PORTE |= 1<<latch_pin;//Latch pin HIGH
PORTE &= ~(1<<latch_pin);//Latch pin LOW
delayMicroseconds(3); //???;
PORTE &= ~(1<<blank_pin);//Blank pin LOW to turn on the LEDs with the new data
//delayMicroseconds(5); //???;

anodeLevel++;//inrement the anode level
level = anodeLevel*2;//increment the level variable by 1, which is used to shift out data, since the next level woudl be the next 1 bytes in the arrays

if(anodeLevel==8)//go back to 0 if max is reached
anodeLevel=0;
if(level==16)//if you hit 16 on level, this means you just sent out all 16 bytes, so go back// QUAN TRONG
level=0;
pinMode(blank_pin, OUTPUT);//moved down here so outputs are all off until the first call of this function
}

inline static uint8_t myTransfer(uint8_t C_data){
  SPDR = C_data;
  asm volatile("nop"); asm volatile("nop");
}

void clearfast ()
{
for (unsigned char j=0; j<32; j++)
        {
        red[0][j]   = 0;
        red[1][j]   = 0;
        red[2][j]   = 0;
        red[3][j]   = 0;
        green[0][j] = 0;
        green[1][j] = 0;
        green[2][j] = 0;
        green[3][j] = 0;
        blue[0][j]  = 0;
        blue[1][j]  = 0;
        blue[2][j]  = 0;
        blue[3][j]  = 0;
        }
}
void loop(){

 for (byte x=0; x<16; x++)
  {
    for (byte y=0; y<16; y++)
      {
        LED(x,y,15,0,0);
        delay(50);
      }
  }
  delay (2000);
for (byte y=0; y<16; y++)
  {
  for (byte x=0; x<16; x++)
  {

        LED(x,y,0,15,0);
        delay(50);
      }
  }
  
  delay (2000);
for (byte x=0; x<16; x++)
  {
    for (byte y=0; y<16; y++)
      {
        LED(x,y,0,0,15);
        delay(50);
      }
  }
  
}

