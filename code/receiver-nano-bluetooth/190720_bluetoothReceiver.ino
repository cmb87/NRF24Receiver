#define VERSION     "\n\nAndroTest V2.0 - @kas2014\ndemo for V5.x App"

// V2.0  changed to pure ASCII Communication Protocol ** not backward compatible **
// V1.4  improved communication errors handling
// V1.3  renamed for publishing, posted on 09/05/2014
// V1.2  Text display   ** not backward compatible **
// V1.1  Integer display
// V1.0  6 buttons + 4 data char implemented

// Demo setup:
// Button #1 controls pin #13 LED
// Button #4 toggle datafield display rate
// Button #5 configured as "push" button (momentary)
// Other buttons display demo message

// Arduino pin#2 to TX BlueTooth module
// Arduino pin#3 to RX BlueTooth module
// make sure your BT board is set @57600 bps
// better remove SoftSerial for PWM based projects

// For Mega 2560:
// remove   #include "SoftwareSerial.h", SoftwareSerial mySerial(2,3);
// search/replace  mySerial  >> Serial1
// pin#18 to RX bluetooth module, pin#19 to TX bluetooth module

#include "SoftwareSerial.h"


////////////////////// BT //////////////////////////

#define    STX          0x02
#define    ETX          0x03
#define    SLOW         750                            // Datafields refresh rate (ms)
#define    FAST         250                             // Datafields refresh rate (ms)
unsigned long lastRecvTime = 0;
int throttle = 0;       
int yaw = 127;

////////////////////// PPM CONFIGURATION//////////////////////////
#define channel_number 6  //set the number of channels
#define sigPin 2  //set PPM signal output pin on the arduino
#define PPM_FrLen 27000  //set the PPM frame length in microseconds (1ms = 1000µs)
#define PPM_PulseLen 400  //set the pulse length
int ppm[channel_number];
//////////////////////////////////////////////////////////////////

SoftwareSerial mySerial(7,6);                           // BlueTooth module: pin#2=TX pin#3=RX
byte cmd[8] = {0, 0, 0, 0, 0, 0, 0, 0};                 // bytes received
byte buttonStatus = 0;                                  // first Byte sent to Android device
long previousMillis = 0;                                // will store last time Buttons status was updated
long sendInterval = SLOW;                               // interval between Buttons status transmission (milliseconds)
String displayStatus = "xxxx";                          // message to Android device


// ================ PPM ===============
void setupPPM() {

  pinMode(sigPin, OUTPUT);
  digitalWrite(sigPin, 0);  //set the PPM signal pin to the default state (off)

  cli();
  TCCR1A = 0; // set entire TCCR1 register to 0
  TCCR1B = 0;

  OCR1A = 100;  // compare match register (not very important, sets the timeout for the first interrupt)
  TCCR1B |= (1 << WGM12);  // turn on CTC mode
  TCCR1B |= (1 << CS11);  // 8 prescaler: 0,5 microseconds at 16mhz
  TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
  sei();
}

// ================ Setup ===============
void setup()  {

  resetData();
  
  setupPPM();
  Serial.begin(57600);
  mySerial.begin(57600);                                // 57600 = max value for softserial  
  Serial.println(VERSION);
  while(mySerial.available())  mySerial.read();         // empty RX buffer

}


// ================ Main loop ===============
void loop() {
 // ---------------- Fail safe ------------------
  unsigned long now = millis();
  if ( now - lastRecvTime > 1000 ) {
    // signal lost?
    resetData();
  }
  
 // ---------------- BT Communication ------------------
 if(mySerial.available())  {                           // data received from smartphone
   lastRecvTime = millis();
   delay(2);
   cmd[0] =  mySerial.read();  
   if(cmd[0] == STX)  {
     int i=1;      
     while(mySerial.available())  {
       delay(1);
       cmd[i] = mySerial.read();
       if(cmd[i]>127 || i>7)                 break;     // Communication error
       if((cmd[i]==ETX) && (i==2 || i==7))   break;     // Button or Joystick data
       i++;
     }
     if     (i==2)          getButtonState(cmd[1]);    // 3 Bytes  ex: < STX "C" ETX >
     else if(i==7)          getJoystickState(cmd);     // 6 Bytes  ex: < STX "200" "180" ETX >
   }
 }

  sendBlueToothData();
}
// ================ Send BT Data ===============
void sendBlueToothData()  {
 static long previousMillis = 0;                            
 long currentMillis = millis();
 if(currentMillis - previousMillis > sendInterval) {   // send data back to smartphone
   previousMillis = currentMillis;

// Data frame transmitted back from Arduino to Android device:
// < 0X02   Buttons state   0X01   DataField#1   0x04   DataField#2   0x05   DataField#3    0x03 >  
// < 0X02      "01011"      0X01     "120.00"    0x04     "-4500"     0x05  "Motor enabled" 0x03 >    // example

// ---------------- GPS ------------------
  // This sketch displays information every time a new sentence is correctly encoded.
 //  while (gpsSerial.available() > 0){
  //   byte gpsData = gpsSerial.read();
  //   gps.encode(gpsSerial.read());
  //   if (gps.location.isUpdated()){
  //     lat = gps.location.lat();
 //      lng = gps.location.lng();
  //   }
 //   }
  // ---------------- GPS ------------------

   mySerial.print((char)STX);                                             // Start of Transmission
   mySerial.print(getButtonStatusString());  mySerial.print((char)0x1);   // buttons status feedback
   mySerial.print(0.0);            mySerial.print((char)0x4);   // datafield #1
   mySerial.print(0.0);            mySerial.print((char)0x5);   // datafield #2
   mySerial.print(displayStatus);                                         // datafield #3
   mySerial.print((char)ETX);                                             // End of Transmission
 }  
}

String getButtonStatusString()  {
 String bStatus = "";
 for(int i=0; i<6; i++)  {
   if(buttonStatus & (B100000 >>i))      bStatus += "1";
   else                                  bStatus += "0";
 }
 return bStatus;
}


void getJoystickState(byte data[8])    {
 int joyX = (data[1]-48)*100 + (data[2]-48)*10 + (data[3]-48);       // obtain the Int from the ASCII representation
 int joyY = (data[4]-48)*100 + (data[5]-48)*10 + (data[6]-48);
 joyX = joyX - 200;                                                  // Offset to avoid
 joyY = joyY - 200;                                                  // transmitting negative numbers

 if(joyX<-100 || joyX>100 || joyY<-100 || joyY>100)     return;      // commmunication error
 
// Your code here ...
    Serial.print("Joystick position:  ");
    Serial.print(joyX);  
    Serial.print(", ");  
    Serial.println(joyY);
    
    ppm[0] = map(throttle, 0, 255, 1000, 2000);
    ppm[1] = map(yaw,      0, 255, 1000, 2000);
    ppm[2] = map(joyY,    -100, 100, 1000, 2000);
    ppm[3] = map(joyX,     -100, 100, 1000, 2000);
    ppm[4] = map(0,     0, 1, 1000, 2000);
    ppm[5] = map(0,     0, 1, 1000, 2000);  
}


void resetData(){
  // 'safe' values to use when no radio input is detected
    ppm[0] = 1000;
    ppm[1] = 1500;
    ppm[2] = 1500;
    ppm[3] = 1500;
    ppm[4] = 1500;
    ppm[5] = 1500;  
}


void getButtonState(int bStatus)  {
 switch (bStatus) {
// -----------------  BUTTON #1  -----------------------
   case 'A':
     buttonStatus |= B000001;        // ON
     Serial.println("\n** Button_1: ON **");
     // your code...      
     displayStatus = "Throttle ++";
     Serial.println(displayStatus);
     throttle+=5;
     if (throttle > 255 ) throttle=250;
     Serial.println(ppm[0]);
     break;
   case 'B':
     buttonStatus &= B111110;        // OFF
     Serial.println("\n** Button_1: OFF **");
     // your code...      
     break;

// -----------------  BUTTON #2  -----------------------
   case 'C':
     buttonStatus |= B000010;        // ON
     Serial.println("\n** Button_2: ON **");
     // your code...      
     displayStatus = "Throttle --";
     Serial.println(displayStatus);
     throttle-=5;
     if (throttle < 0 ) throttle=0;
     Serial.println(throttle);
     break;
   case 'D':
     buttonStatus &= B111101;        // OFF
     Serial.println("\n** Button_2: OFF **");
     // your code...      
     break;

// -----------------  BUTTON #3  -----------------------
   case 'E':
     buttonStatus |= B000100;        // ON
     Serial.println("\n** Button_3: ON **");
     // your code...      
     displayStatus = "Yaw++"; // Demo text message
     Serial.println(displayStatus);
     yaw+=10;
     if (yaw > 255 ) yaw=250;
     Serial.println(yaw);
     break;
   case 'F':
     buttonStatus &= B111011;      // OFF
     Serial.println("\n** Button_3: OFF **");
     // your code...      
     break;

// -----------------  BUTTON #4  -----------------------
   case 'G':
     buttonStatus |= B001000;       // ON
     Serial.println("\n** Button_4: ON **");
     // your code...      
     displayStatus = "Datafield update <FAST>";
     Serial.println(displayStatus);
     yaw-=10;
     if (yaw < 0 ) yaw=0;
     Serial.println(yaw);
     break;
   case 'H':
     buttonStatus &= B110111;    // OFF
     Serial.println("\n** Button_4: OFF **");
     // your code...      
     displayStatus = "Datafield update <SLOW>";
     Serial.println(displayStatus);
    break;

// -----------------  BUTTON #5  -----------------------
   case 'I':           // configured as momentary button
//      buttonStatus |= B010000;        // ON
     Serial.println("\n** Button_5: ++ pushed ++ **");
     // your code...      
     displayStatus = "MaxPower";
     throttle=255;
     Serial.println(displayStatus);
     break;
   case 'J':
     buttonStatus &= B101111;        // OFF
//     // your code...    
     displayStatus = "MinPower";
     throttle=0;
     Serial.println(displayStatus);
     break;

// -----------------  BUTTON #6  -----------------------
   case 'K':
     buttonStatus |= B100000;        // ON
     Serial.println("\n** Button_6: ON **");
     // your code...      
     displayStatus = "YawNeutral";
     yaw=127;
     Serial.println(displayStatus);
    break;
   case 'L':
     buttonStatus &= B011111;        // OFF
     Serial.println("\n** Button_6: OFF **");
     // your code...      
     displayStatus = "Button6 <OFF>";
     break;
 }
// ---------------------------------------------------------------
}

// --------------------------- ISR ------------------------
#define clockMultiplier 2 // set this to 2 if you are using a 16MHz arduino, leave as 1 for an 8MHz arduino

ISR(TIMER1_COMPA_vect){
  static boolean state = true;

  TCNT1 = 0;

  if ( state ) {
    //end pulse
    PORTD = PORTD & ~B00000100; // turn pin 2 off. Could also use: digitalWrite(sigPin,0)
    OCR1A = PPM_PulseLen * clockMultiplier;
    state = false;
  }
  else {
    //start pulse
    static byte cur_chan_numb;
    static unsigned int calc_rest;

    PORTD = PORTD | B00000100; // turn pin 2 on. Could also use: digitalWrite(sigPin,1)
    state = true;

    if(cur_chan_numb >= channel_number) {
      cur_chan_numb = 0;
      calc_rest += PPM_PulseLen;
      OCR1A = (PPM_FrLen - calc_rest) * clockMultiplier;
      calc_rest = 0;
    }
    else {
      OCR1A = (ppm[cur_chan_numb] - PPM_PulseLen) * clockMultiplier;
      calc_rest += ppm[cur_chan_numb];
      cur_chan_numb++;
    }     
  }
}
