//#include <IRremoteInt.h>
//#include <IRremoteTools.h>
//#include <IRremote.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x41);
Adafruit_PWMServoDriver led1s = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver led2s = Adafruit_PWMServoDriver(0x41);
Adafruit_PWMServoDriver led1d = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver led2d = Adafruit_PWMServoDriver(0x41);

Adafruit_PWMServoDriver* bp[] = {&pwm2, &pwm2, &pwm2, &pwm2, &pwm2};
Adafruit_PWMServoDriver* bls[] = {&led1s, &led2s, &led1s, &led1s, &led1s, &led1s, &led1s, &led1s, &led1s};
Adafruit_PWMServoDriver* bld[] = {&led2d, &led1d, &led1d, &led1d, &led1d, &led1d, &led1d, &led1d, &led1d};

byte button_total = 9;
int button_id[] = {13,8,12,11,6,5,10,7,9};            // toggle buttons
int point_id[] = {15,14,13,12,11,10,9,8,7};             // points connected to Adafruit 16-channel PWM servo driver
int p_direction[] = {150,150,150,150,150,150,150,150,150};      // actual position
int p_straight[] = {150,150,150,150,150,150,150,150,150};       // value to move the point in straight position
int p_diverging[] = {270,270,270,270,270,270,270,270,270};      // value to move the point in diverging position
int l_straight[] = {3,1,8,1,0,5,13,14,15};             // leds, straight position
int l_diverging[] = {3,4,2,7,6,12,9,10,11};            // leds, diverging postition

int reading;            // current reading from the input pin

//Funzione antirimbalzo
 
int debounceDelay = 10;
 
boolean debounce(int pin)
{
  boolean state;
  boolean previousState;
  previousState = digitalRead(pin);
  for(int counter=0; counter < debounceDelay; counter++) {
    delay(1);
    state = digitalRead(pin);
    if( state != previousState) {
      counter = 0;
      previousState = state; }
  }
  return state;
}

void setup()
{
  Serial.begin(9600);
  pwm1.begin();
  pwm1.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  pwm2.begin();
  pwm2.setPWMFreq(60);  // Analog servos run at ~60 Hz updates


  for(byte i = 0; i < button_total; i++)
  {
    pinMode(l_straight[i], OUTPUT);
    pinMode(button_id[i], INPUT);
    digitalWrite(button_id[i],HIGH);
    bp[i]->setPWM(point_id[i], 0, p_straight[i]);
    delay(200);
    bp[i]->setPWM(point_id[i], 0, 0);              // turn off servo
    bld[i]->setPWM(l_diverging[i], 0, 0);
    bls[i]->setPWM(l_straight[i], 4096, 0);
  }
} 

void loop()
{
   for(byte index = 0; index < button_total; index++)
  {
    reading = debounce(button_id[index]); // read input
    if (reading == LOW)                   // check if a button was pressed
    {
          //Serial.println(reading);
//    Serial.print("Read Input "); 
//    Serial.print(button_id[index]);
//    Serial.print(": ");   
//    Serial.println(reading);  
//    delay(1000);
          toggle_point(index);
          
    }
    //previous = HIGH;
  }
}


void toggle_point(byte which)
{
    if (p_direction[which] == p_straight[which])
      {
      p_direction[which] = p_diverging[which];
      for (uint16_t pulselen = p_straight[which]; pulselen <= p_diverging[which]; pulselen++) {
        bp[which]->setPWM(point_id[which], 0, pulselen);
//                  Serial.print("Pulselen = ");
//                  Serial.println(pulselen);
        delay(5);
                  if (pulselen > (p_diverging[which]-(p_diverging[which]*.35)))
                  {
                    bld[which]->setPWM(l_diverging[which], 4096, 0);
                    }    
                  if (pulselen > (p_straight[which]+(p_straight[which]*.35)))
                  {
                    bls[which]->setPWM(l_straight[which], 0, 0);
                  }
      }
      bp[which]->setPWM(point_id[which], 0, 0);              // turn off servo
      }
    else
      {
      p_direction[which] = p_straight[which];
                  Serial.print("Turnout number ");
                  Serial.print(point_id[which]);
                  Serial.print(" is diverged - ");
                  Serial.println(p_diverging[which]);
                  //delay(1000);
      for (uint16_t pulselen = p_diverging[which]; pulselen >= p_straight[which]; pulselen--) {
        bp[which]->setPWM(point_id[which], 0, pulselen);
//                  Serial.print("Pulselen = ");
//                  Serial.println(pulselen);
        delay(5);
                  if (pulselen < (p_diverging[which]-(p_diverging[which]*.35)))
                  {
                    bld[which]->setPWM(l_diverging[which], 0, 0);
                  }
                  if (pulselen < (p_straight[which]+(p_straight[which]*.35)))
                  {
                    bls[which]->setPWM(l_straight[which], 4096, 0);
                  }    
      }
      bp[which]->setPWM(point_id[which], 0, 0);              // turn off servo
      }
}

