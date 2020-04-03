/*
         Arduino based RC Hovercraft
   == Receiver Code - ESC and Servo Control =
  by Dejan, www.HowToMechatronics.com
  Library: TMRh20/RF24, https://github.com/tmrh20/RF24/
  https://howtomechatronics.com/projects/diy-arduino-based-rc-hovercraft/
*/
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>
#define led 10
RF24 radio(8, 9);   // nRF24L01 (CE, CSN)
const byte address[6] = "00001";
unsigned long lastReceiveTime = 0;
unsigned long currentTime = 0;
Servo esc1;  // create servo object to control the ESC
Servo esc2;
Servo servo1;
int esc1Value, esc2Value, servo1Value;
// Max size of this struct is 32 bytes - NRF24L01 buffer limit
struct Data_Package {
  byte j1PotX;
  byte j1PotY;
  byte j1Button;
  byte j2PotX;
  byte j2PotY;
  byte j2Button;
  byte pot1;
  byte pot2;
  byte tSwitch1;
  byte tSwitch2;
  byte button1;
  byte button2;
  byte button3;
  byte button4;
};
Data_Package data; //Create a variable with the above structure
void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.startListening(); //  Set the module as receiver
  resetData();
  esc1.attach(7);
  esc2.attach(6);
  servo1.attach(5);
  pinMode(led, OUTPUT);
}
void loop() {
  // Check whether we keep receving data, or we have a connection between the two modules
  currentTime = millis();
  if ( currentTime - lastReceiveTime > 1000 ) { // If current time is more then 1 second since we have recived the last data, that means we have lost connection
    resetData(); // If connection is lost, reset the data. It prevents unwanted behavior, for example if a drone jas a throttle up, if we lose connection it can keep flying away if we dont reset the function
  }
  // Check whether there is data to be received
  if (radio.available()) {
    radio.read(&data, sizeof(Data_Package)); // Read the whole data and store it into the 'data' structure
    lastReceiveTime = millis(); // At this moment we have received the data
  }
  // Controlling servos
  servo1Value = map(data.j2PotX, 0, 255, 0, 50);
  servo1.write(servo1Value);
  // Controlling brushless motor with ESC
  // Lift propeller
  esc1Value = map(data.pot1, 0, 255, 1000, 2000); // Map the receiving value form 0 to 255 to 0 1000 to 2000, values used for controlling ESCs
  esc1.writeMicroseconds(esc1Value); // Send the PWM control singal to the ESC
  
  // Thrust propeller
  esc2Value = constrain(data.j1PotY, 130, 255); // Joysticks stays in middle. So we only need values the upper values from 130 to 255
  esc2Value = map(esc2Value, 130, 255, 1000, 2000);
  esc2.writeMicroseconds(esc2Value);
  // Monitor the battery voltage
  int sensorValue = analogRead(A0);
  float voltage = sensorValue * (5.00 / 1023.00) * 3; // Convert the reading values from 5v to suitable 12V i
  Serial.println(voltage);
  // If voltage is below 11V turn on the LED
  if (voltage < 11) {
    digitalWrite(led, HIGH);
  }
  else {
    digitalWrite(led, LOW);
  }
}
void resetData() {
  // Reset the values when there is no radio connection - Set initial default values
  data.j1PotX = 127;
  data.j1PotY = 127;
  data.j2PotX = 127;
  data.j2PotY = 127;
  data.j1Button = 1;
  data.j2Button = 1;
  data.pot1 = 1;
  data.pot2 = 1;
  data.tSwitch1 = 1;
  data.tSwitch2 = 1;
  data.button1 = 1;
  data.button2 = 1;
  data.button3 = 1;
  data.button4 = 1;
}