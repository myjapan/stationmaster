//#include <IRremoteInt.h>
//#include <IRremoteTools.h>
//#include <IRremote.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

int point_id[] = {0,1,2,3,4,5,6};
int button_id[] = {2,3,4,5,6,7,8};
int p_direction[] = {150,150,150,150,150,150,150};
int p_straight[] = {150,150,150,150,150,150,150};
int p_dev[] = {270,270,270,270,270,270,270};
int l_straight[] = {7,8,9,10,11,12,13};

int button_total = 7;


uint8_t inPin = 2;          // the number of the input pin. Il bottone è collegato tra il pin di ingresso e la massa
uint8_t direzioneP0=150;
int MINP0 = 150;
int MAXP0 = 270;        // the current state of the output pin
int reading;            // the current reading from the input pin
int previous = HIGH;    // the previous reading from the input pin

                        // the follow variables are long's because the time, measured in miliseconds,
                        // will quickly become a bigger number than can be stored in an int.
long time = 0;          // the last time the output pin was toggled
byte debounce = 200;    // the debounce time, increase if the output flickers
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

void setup()
{
  Serial.begin(4800);
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  
  pinMode(inPin, INPUT_PULLUP);
  pwm.setPWM(0, 0, direzioneP0);


 }

void loop()
{

   for(int index = 0; index < button_total; index++)  
  {  
    reading = digitalRead(button_id[index]);  // legge il valore di input  
    if (reading == HIGH && previous == LOW && millis() - time > debounce)                              // controlla se il pulsante è premuto  
    {  
      if (index == 0)  
      {  
        time = millis();
        toggle_point(point_id[index]);
//        blink1(pinLed[index]);  
      }  
      if (index == 1)  
      {  
        time = millis();
//        blink2(pinLed[indice]);  
      }  
      if (index == 2)  
      {  
        time = millis();
//        blink3(pinLed[indice]);  
      }  
      if (index == 3)  
      {  
        time = millis();
//        blink4(pinLed[indice]);  
      }  
    }
    previous = reading;  
//    else  
//    {  
//      digitalWrite(l_straight[index], LOW);          // spegne il LED  
//    }  
  }

//  reading = digitalRead(inPin);

                        // if the input just went from LOW and HIGH and we've waited long enough
                        // to ignore any noise on the circuit, toggle the output pin and remember
                        // the time
//  if (reading == HIGH && previous == LOW && millis() - time > debounce) {
    
}    

void toggle_point(int which)
{    
    if (p_direction[which] == p_straight[which])
      {
      p_direction[which] = p_dev[which] ;
      //pwm.setPWM(0, 0, direzioneP0);
      for (uint16_t pulselen = p_straight[which]; pulselen < p_dev[which]; pulselen++) {
      pwm.setPWM(point_id[which], 0, pulselen);
      delay(6);
      }
      delay(500);
      pwm.setPWM(point_id[which], 0, 0); 
      }
    else
      {
        p_direction[which] = p_straight[which];
        //pwm.setPWM(0, 0, direzioneP0);
      for (uint16_t pulselen = p_dev[which]; pulselen > p_straight[which]; pulselen--) {
      pwm.setPWM(point_id[which], 0, pulselen);
      delay(6);
      }  
      delay(500);
        pwm.setPWM(point_id[which], 0, 0); 
      }
      Serial.println(p_direction[which]);
    time = millis();    
}
//Serial.println(direzioneP0);

//pwm.setPWM(0, 0, 0);  // turn off PWM signal to channel 0
//myservo.write(direzione,7,false); 
//if (myservo.read() > 90)      //opzionale: led indicazione di stato dello scambio
//   digitalWrite(13, HIGH);
//else
//   digitalWrite(13, LOW);

// previous = reading;
//}

