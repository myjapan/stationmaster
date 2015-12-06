//#include <IRremoteInt.h>
//#include <IRremoteTools.h>
//#include <IRremote.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// const int servoPin = 9; // the digital pin used for the servo

int inPin = 2;          // the number of the input pin. Il bottone è collegato tra il pin di ingresso e la massa

int direzioneP0=375;
int MINP0 = 375;
int MAXP0 = 600;        // the current state of the output pin
int reading;            // the current reading from the input pin
int previous = HIGH;    // the previous reading from the input pin

                        // the follow variables are long's because the time, measured in miliseconds,
                        // will quickly become a bigger number than can be stored in an int.
long time = 0;          // the last time the output pin was toggled
long debounce = 200;    // the debounce time, increase if the output flickers
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

void setup()
{
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  
  pinMode(inPin, INPUT_PULLUP);
  pwm.setPWM(0, 0, direzioneP0);


 }

void loop()
{
  reading = digitalRead(inPin);

                        // if the input just went from LOW and HIGH and we've waited long enough
                        // to ignore any noise on the circuit, toggle the output pin and remember
                        // the time
  if (reading == HIGH && previous == LOW && millis() - time > debounce) {
    if (direzioneP0 == MINP0)
      direzioneP0 = MAXP0 ;
    else
      direzioneP0 = MINP0;

    time = millis();    
  }

pwm.setPWM(0, 0, direzioneP0);
//pwm.setPWM(0, 0, 0);  // turn off PWM signal to channel 0
//myservo.write(direzione,7,false); 
//if (myservo.read() > 90)      //opzionale: led indicazione di stato dello scambio
//   digitalWrite(13, HIGH);
//else
//   digitalWrite(13, LOW);

  previous = reading;
}
