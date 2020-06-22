/*  
 * Check:  http://www.electronoobs.com/eng_robotica_tut5_2_1.php
 * 
 * 
A basic receiver test for the nRF24L01 module to receive 6 channels send a ppm sum
with all of them on digital pin D2.
Install NRF24 library before you compile
Please, like, share and subscribe on my https://www.youtube.com/c/ELECTRONOOBS
 */

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

////////////////////// PPM CONFIGURATION//////////////////////////
#define channel_number 6  //set the number of channels
#define sigPin 2  //set PPM signal output pin on the arduino
#define PPM_FrLen 27000  //set the PPM frame length in microseconds (1ms = 1000µs)
#define PPM_PulseLen 400  //set the pulse length
//////////////////////////////////////////////////////////////////

int ppm[channel_number];
int pitchtrim = -5;

const byte address[6] = "00001";


RF24 radio(9, 10); // nRF24L01 (CE, CSN)

// The sizeof this struct should not exceed 32 bytes
struct Data_Package {
  byte j1PotX;
  byte j1PotY;
  byte j1Button;
  byte j2PotX;
  byte j2PotY;
  byte j2Button;
};

Data_Package data;

void resetData() 
{
  // 'safe' values to use when no radio input is detected
  data.j1PotX = 127;
  data.j1PotY = 64; // should correspond to approximatively 1250 for throttle
  data.j1Button = 0;
  data.j2PotX = 127;
  data.j2PotY = 127;
  data.j2Button= 0;
  
  setPPMValuesFromData();
}


// Mutliwii THROTTLE,YAW,ROLL,PITCH
void setPPMValuesFromData()
{
  ppm[0] = map(data.j1PotX,     0, 255, 1000, 2000);
  ppm[1] = map(data.j1PotY,     0, 255, 1000, 2000);
  ppm[2] = map(data.j2PotY+pitchtrim,     0, 255, 1000, 2000);
  ppm[3] = map(data.j2PotX,     0, 255, 1000, 2000);
  ppm[4] = map(data.j1Button,   0, 1, 1000, 2000);  
  ppm[5] = map(data.j2Button,   0, 1, 1000, 2000);  
  }

/**************************************************/

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

void setup()
{  
  resetData();
  setupPPM();
  
  // Set up radio module
  radio.begin();
  radio.openReadingPipe(0, address);
  //radio.setPALevel(RF24_PA_MIN);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.startListening();
  Serial.begin(115200);
  Serial.println("Receiver up and running");
}

/**************************************************/

unsigned long lastRecvTime = 0;

void recvData()
{  
  while ( radio.available() ) {        
    radio.read(&data, sizeof(Data_Package));
    lastRecvTime = millis();
//    Serial.println(radio.available());
//    Serial.println(radio.isChipConnected());
    Serial.println("");
    Serial.print(data.j1PotX);
    Serial.print(" ");
    Serial.print(data.j1PotY);
    Serial.print(" ");
    Serial.print(data.j1Button);
    Serial.print(" ");
    Serial.print(data.j2PotX);
    Serial.print(" ");
    Serial.print(data.j2PotY+pitchtrim);
    Serial.print(" ");
    Serial.print(data.j2Button);
    Serial.println("");
    Serial.flush();
  }
}

/**************************************************/

void loop()
{
  // ---------------- RADIO -----------------
  recvData();
  unsigned long now = millis();
  if ( now - lastRecvTime > 1000 ) {
    // signal lost?
    resetData();
  }
  
  setPPMValuesFromData();
}

/**************************************************/

//#error Delete this line befor you cahnge the value (clockMultiplier) below
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
