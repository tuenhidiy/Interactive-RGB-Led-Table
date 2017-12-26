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

byte Zone00[4][2] = {{6, 0}, {7, 0}, {6, 1}, {7, 1}};
byte Zone01[4][2] = {{4, 0}, {5, 0}, {4, 1}, {5, 1}};
byte Zone02[4][2] = {{2, 0}, {3, 0}, {2, 1}, {3, 1}};
byte Zone03[4][2] = {{0, 0}, {1, 0}, {0, 1}, {1, 1}};

byte Zone04[4][2] = {{6, 2}, {7, 2}, {6, 3}, {7, 3}};
byte Zone05[4][2] = {{4, 2}, {5, 2}, {4, 3}, {5, 3}};
byte Zone06[4][2] = {{2, 2}, {3, 2}, {2, 3}, {3, 3}};
byte Zone07[4][2] = {{0, 2}, {1, 2}, {0, 3}, {1, 3}};

byte Zone08[4][2] = {{6, 4}, {7, 4}, {6, 5}, {7, 5}};
byte Zone09[4][2] = {{4, 4}, {5, 4}, {4, 5}, {5, 5}};
byte Zone10[4][2] = {{2, 4}, {3, 4}, {2, 5}, {3, 5}};
byte Zone11[4][2] = {{0, 4}, {1, 4}, {0, 5}, {1, 5}};

byte Zone12[4][2] = {{6, 6}, {7, 6}, {6, 7}, {7, 7}};
byte Zone13[4][2] = {{4, 6}, {5, 6}, {4, 7}, {5, 7}};
byte Zone14[4][2] = {{2, 6}, {3, 6}, {2, 7}, {3, 7}};
byte Zone15[4][2] = {{0, 6}, {1, 6}, {0, 7}, {1, 7}};

byte Zone16[4][2] = {{14, 0}, {15, 0}, {14, 1}, {15, 1}};
byte Zone17[4][2] = {{12, 0}, {13, 0}, {12, 1}, {13, 1}};
byte Zone18[4][2] = {{10, 0}, {11, 0}, {10, 1}, {11, 1}};
byte Zone19[4][2] = {{8, 0}, {9, 0}, {8, 1}, {9, 1}};

byte Zone20[4][2] = {{14, 2}, {15, 2}, {14, 3}, {15, 3}};
byte Zone21[4][2] = {{12, 2}, {13, 2}, {12, 3}, {13, 3}};
byte Zone22[4][2] = {{10, 2}, {11, 2}, {10, 3}, {11, 3}};
byte Zone23[4][2] = {{8, 2}, {9, 2}, {8, 3}, {9, 3}};

byte Zone24[4][2] = {{14, 4}, {15, 4}, {14, 5}, {15, 5}};
byte Zone25[4][2] = {{12, 4}, {13, 4}, {12, 5}, {13, 5}};
byte Zone26[4][2] = {{10, 4}, {11, 4}, {10, 5}, {11, 5}};
byte Zone27[4][2] = {{8, 4}, {9, 4}, {8, 5}, {9, 5}};

byte Zone28[4][2] = {{14, 6}, {15, 6}, {14, 7}, {15, 7}};
byte Zone29[4][2] = {{12, 6}, {13, 6}, {12, 7}, {13, 7}};
byte Zone30[4][2] = {{10, 6}, {11, 6}, {10, 7}, {11, 7}};
byte Zone31[4][2] = {{8, 6}, {9, 6}, {8, 7}, {9, 7}};

byte Zone32[4][2] = {{6, 8}, {7, 8}, {6, 9}, {7, 9}};
byte Zone33[4][2] = {{4, 8}, {5, 8}, {4, 9}, {5, 9}};
byte Zone34[4][2] = {{2, 8}, {3, 8}, {2, 9}, {3, 9}};
byte Zone35[4][2] = {{0, 8}, {1, 8}, {0, 9}, {1, 9}};

byte Zone36[4][2] = {{6, 10}, {7, 10}, {6, 11}, {7, 11}};
byte Zone37[4][2] = {{4, 10}, {5, 10}, {4, 11}, {5, 11}};
byte Zone38[4][2] = {{2, 10}, {3, 10}, {2, 11}, {3, 11}};
byte Zone39[4][2] = {{0, 10}, {1, 10}, {0, 11}, {1, 11}};

byte Zone40[4][2] = {{6, 12}, {7, 12}, {6, 13}, {7, 13}};
byte Zone41[4][2] = {{4, 12}, {5, 12}, {4, 13}, {5, 13}};
byte Zone42[4][2] = {{2, 12}, {3, 12}, {2, 13}, {3, 13}};
byte Zone43[4][2] = {{0, 12}, {1, 12}, {0, 13}, {1, 13}};

byte Zone44[4][2] = {{6, 14}, {7, 14}, {6, 15}, {7, 15}};
byte Zone45[4][2] = {{4, 14}, {5, 14}, {4, 15}, {5, 15}};
byte Zone46[4][2] = {{2, 14}, {3, 14}, {2, 15}, {3, 15}};
byte Zone47[4][2] = {{0, 14}, {1, 14}, {0, 15}, {1, 15}};

byte Zone48[4][2] = {{14, 8}, {15, 8}, {14, 9}, {15, 9}};
byte Zone49[4][2] = {{12, 8}, {13, 8}, {12, 9}, {13, 9}};
byte Zone50[4][2] = {{10, 8}, {11, 8}, {10, 9}, {11, 9}};
byte Zone51[4][2] = {{8, 8}, {9, 8}, {8, 9}, {9, 9}};

byte Zone52[4][2] = {{14, 10}, {15, 10}, {14, 11}, {15, 11}};
byte Zone53[4][2] = {{12, 10}, {13, 10}, {12, 11}, {13, 11}};
byte Zone54[4][2] = {{10, 10}, {11, 10}, {10, 11}, {11, 11}};
byte Zone55[4][2] = {{8, 10}, {9, 10}, {8, 11}, {9, 11}};

byte Zone56[4][2] = {{14, 12}, {15, 12}, {14, 13}, {15, 13}};
byte Zone57[4][2] = {{12, 12}, {13, 12}, {12, 13}, {13, 13}};
byte Zone58[4][2] = {{10, 12}, {11, 12}, {10, 13}, {11, 13}};
byte Zone59[4][2] = {{8, 12}, {9, 12}, {8, 13}, {9, 13}};

byte Zone60[4][2] = {{14, 14}, {15, 14}, {14, 15}, {15, 15}};
byte Zone61[4][2] = {{12, 14}, {13, 14}, {12, 15}, {13, 15}};
byte Zone62[4][2] = {{10, 14}, {11, 14}, {10, 15}, {11, 15}};
byte Zone63[4][2] = {{8, 14}, {9, 14}, {8, 15}, {9, 15}};


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

void Interactive(){
  if ( (unsigned long) (micros() - samplingtime) > 200  )
  {
  // now read analog inputs and store sense data into sense array
  ir_sense_data[0 + ir_group_adder] = analogRead(A0);
  ir_sense_data[8 + ir_group_adder] = analogRead(A1);
  ir_sense_data[16 + ir_group_adder] = analogRead(A2);
  ir_sense_data[24 + ir_group_adder] = analogRead(A3);
  ir_sense_data[32 + ir_group_adder] = analogRead(A4);
  ir_sense_data[40 + ir_group_adder] = analogRead(A5);
  ir_sense_data[48 + ir_group_adder] = analogRead(A6);
  ir_sense_data[56 + ir_group_adder] = analogRead(A7);

  // update sensor group and mux selects for next time through
  if (ir_group < 7 ) {
    ir_group++;
  } else {
    ir_group = 0;
  }

  ir_group_adder = ir_group;
  
  digitalWrite(sense_select1, bitRead(ir_group, 2));    // sense mux MSB
  digitalWrite(sense_select2, bitRead(ir_group, 1));
  digitalWrite(sense_select3, bitRead(ir_group, 0));    // sense mux LSB
  samplingtime = micros();
  } 
    for (byte i=0; i<64; i++){
    if (ir_sense_data[i] < LimitSense)   

      ClearLightZone(i);

    else
    {
      SetLightZone(i, 15, 0, 0);

    }
  }  
}


void SetLightZone(byte Zone, byte R, byte G, byte B)
{
switch (Zone)
  {
    case 0:   DrawLight(Zone00, R, G, B); break;
    case 1:   DrawLight(Zone01, R, G, B); break;
    case 2:   DrawLight(Zone02, R, G, B); break;
    case 3:   DrawLight(Zone03, R, G, B); break;
    case 4:   DrawLight(Zone04, R, G, B); break;
    case 5:   DrawLight(Zone05, R, G, B); break;
    case 6:   DrawLight(Zone06, R, G, B); break;
    case 7:   DrawLight(Zone07, R, G, B); break;
    case 8:   DrawLight(Zone08, R, G, B); break;
    case 9:   DrawLight(Zone09, R, G, B); break;
    case 10:  DrawLight(Zone10, R, G, B); break;
    case 11:  DrawLight(Zone11, R, G, B); break;
    case 12:  DrawLight(Zone12, R, G, B); break;
    case 13:  DrawLight(Zone13, R, G, B); break;
    case 14:  DrawLight(Zone14, R, G, B); break;
    case 15:  DrawLight(Zone15, R, G, B); break;
    case 16:  DrawLight(Zone16, R, G, B); break;
    case 17:  DrawLight(Zone17, R, G, B); break;
    case 18:  DrawLight(Zone18, R, G, B); break;
    case 19:  DrawLight(Zone19, R, G, B); break;
    case 20:  DrawLight(Zone20, R, G, B); break;
    case 21:  DrawLight(Zone21, R, G, B); break;
    case 22:  DrawLight(Zone22, R, G, B); break;
    case 23:  DrawLight(Zone23, R, G, B); break;
    case 24:  DrawLight(Zone24, R, G, B); break;
    case 25:  DrawLight(Zone25, R, G, B); break;
    case 26:  DrawLight(Zone26, R, G, B); break;
    case 27:  DrawLight(Zone27, R, G, B); break;
    case 28:  DrawLight(Zone28, R, G, B); break;
    case 29:  DrawLight(Zone29, R, G, B); break;
    case 30:  DrawLight(Zone30, R, G, B); break;
    case 31:  DrawLight(Zone31, R, G, B); break;
    case 32:  DrawLight(Zone32, R, G, B); break;
    case 33:  DrawLight(Zone33, R, G, B); break;
    case 34:  DrawLight(Zone34, R, G, B); break;
    case 35:  DrawLight(Zone35, R, G, B); break;
    case 36:  DrawLight(Zone36, R, G, B); break;
    case 37:  DrawLight(Zone37, R, G, B); break;
    case 38:  DrawLight(Zone38, R, G, B); break;
    case 39:  DrawLight(Zone39, R, G, B); break;
    case 40:  DrawLight(Zone40, R, G, B); break;
    case 41:  DrawLight(Zone41, R, G, B); break;
    case 42:  DrawLight(Zone42, R, G, B); break;
    case 43:  DrawLight(Zone43, R, G, B); break;
    case 44:  DrawLight(Zone44, R, G, B); break;
    case 45:  DrawLight(Zone45, R, G, B); break;
    case 46:  DrawLight(Zone46, R, G, B); break;
    case 47:  DrawLight(Zone47, R, G, B); break;
    case 48:  DrawLight(Zone48, R, G, B); break;
    case 49:  DrawLight(Zone49, R, G, B); break;
    case 50:  DrawLight(Zone50, R, G, B); break;
    case 51:  DrawLight(Zone51, R, G, B); break;
    case 52:  DrawLight(Zone52, R, G, B); break;
    case 53:  DrawLight(Zone53, R, G, B); break;
    case 54:  DrawLight(Zone54, R, G, B); break;
    case 55:  DrawLight(Zone55, R, G, B); break;
    case 56:  DrawLight(Zone56, R, G, B); break;
    case 57:  DrawLight(Zone57, R, G, B); break;
    case 58:  DrawLight(Zone58, R, G, B); break;
    case 59:  DrawLight(Zone59, R, G, B); break;
    case 60:  DrawLight(Zone60, R, G, B); break;
    case 61:  DrawLight(Zone61, R, G, B); break;
    case 62:  DrawLight(Zone62, R, G, B); break;
    case 63:  DrawLight(Zone63, R, G, B); break;    
  }
}

void ClearLightZone(byte Zone)
{

switch (Zone)
  {
    case 0:   DrawLight(Zone00, 0, 0, 0); break;
    case 1:   DrawLight(Zone01, 0, 0, 0); break;
    case 2:   DrawLight(Zone02, 0, 0, 0); break;
    case 3:   DrawLight(Zone03, 0, 0, 0); break;
    case 4:   DrawLight(Zone04, 0, 0, 0); break;
    case 5:   DrawLight(Zone05, 0, 0, 0); break;
    case 6:   DrawLight(Zone06, 0, 0, 0); break;
    case 7:   DrawLight(Zone07, 0, 0, 0); break;
    case 8:   DrawLight(Zone08, 0, 0, 0); break;
    case 9:   DrawLight(Zone09, 0, 0, 0); break;
    case 10:  DrawLight(Zone10, 0, 0, 0); break;
    case 11:  DrawLight(Zone11, 0, 0, 0); break;
    case 12:  DrawLight(Zone12, 0, 0, 0); break;
    case 13:  DrawLight(Zone13, 0, 0, 0); break;
    case 14:  DrawLight(Zone14, 0, 0, 0); break;
    case 15:  DrawLight(Zone15, 0, 0, 0); break;
    case 16:  DrawLight(Zone16, 0, 0, 0); break;
    case 17:  DrawLight(Zone17, 0, 0, 0); break;
    case 18:  DrawLight(Zone18, 0, 0, 0); break;
    case 19:  DrawLight(Zone19, 0, 0, 0); break;
    case 20:  DrawLight(Zone20, 0, 0, 0); break;
    case 21:  DrawLight(Zone21, 0, 0, 0); break;
    case 22:  DrawLight(Zone22, 0, 0, 0); break;
    case 23:  DrawLight(Zone23, 0, 0, 0); break;
    case 24:  DrawLight(Zone24, 0, 0, 0); break;
    case 25:  DrawLight(Zone25, 0, 0, 0); break;
    case 26:  DrawLight(Zone26, 0, 0, 0); break;
    case 27:  DrawLight(Zone27, 0, 0, 0); break;
    case 28:  DrawLight(Zone28, 0, 0, 0); break;
    case 29:  DrawLight(Zone29, 0, 0, 0); break;
    case 30:  DrawLight(Zone30, 0, 0, 0); break;
    case 31:  DrawLight(Zone31, 0, 0, 0); break;  
    case 32:  DrawLight(Zone32, 0, 0, 0); break;
    case 33:  DrawLight(Zone33, 0, 0, 0); break;
    case 34:  DrawLight(Zone34, 0, 0, 0); break;
    case 35:  DrawLight(Zone35, 0, 0, 0); break;
    case 36:  DrawLight(Zone36, 0, 0, 0); break;
    case 37:  DrawLight(Zone37, 0, 0, 0); break;
    case 38:  DrawLight(Zone38, 0, 0, 0); break;
    case 39:  DrawLight(Zone39, 0, 0, 0); break;
    case 40:  DrawLight(Zone40, 0, 0, 0); break;
    case 41:  DrawLight(Zone41, 0, 0, 0); break;
    case 42:  DrawLight(Zone42, 0, 0, 0); break;
    case 43:  DrawLight(Zone43, 0, 0, 0); break;
    case 44:  DrawLight(Zone44, 0, 0, 0); break;
    case 45:  DrawLight(Zone45, 0, 0, 0); break;
    case 46:  DrawLight(Zone46, 0, 0, 0); break;
    case 47:  DrawLight(Zone47, 0, 0, 0); break;
    case 48:  DrawLight(Zone48, 0, 0, 0); break;
    case 49:  DrawLight(Zone49, 0, 0, 0); break;
    case 50:  DrawLight(Zone50, 0, 0, 0); break;
    case 51:  DrawLight(Zone51, 0, 0, 0); break;
    case 52:  DrawLight(Zone52, 0, 0, 0); break;
    case 53:  DrawLight(Zone53, 0, 0, 0); break;
    case 54:  DrawLight(Zone54, 0, 0, 0); break;
    case 55:  DrawLight(Zone55, 0, 0, 0); break;
    case 56:  DrawLight(Zone56, 0, 0, 0); break;
    case 57:  DrawLight(Zone57, 0, 0, 0); break;
    case 58:  DrawLight(Zone58, 0, 0, 0); break;
    case 59:  DrawLight(Zone59, 0, 0, 0); break;
    case 60:  DrawLight(Zone60, 0, 0, 0); break;
    case 61:  DrawLight(Zone61, 0, 0, 0); break;
    case 62:  DrawLight(Zone62, 0, 0, 0); break;
    case 63:  DrawLight(Zone63, 0, 0, 0); break; 
  }
}

void DrawLight(byte zone_dots[4][2], byte R, byte G, byte B)
{
  for (int i = 0; i < 4; i++)
  {
    LED(zone_dots[i][0], zone_dots[i][1], R, G, B);
    //delay_ms(10);
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

clearfast();
while(1) {
Interactive();
}; 
  
}

