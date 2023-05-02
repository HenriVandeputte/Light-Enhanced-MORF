//*************************************************************************
//*** SOFT ROBOTIC ACTUATOR AND LIGHT CONTROL WITH ARDUINO MOTOR SHIELD ***
//***              Jonas Jørgensen & Henri Vandeputte 2018              ***
//*************************************************************************
// Based on "Simple Motor Shield sketch" (June 2012, Open Source / Public Domain) by arduino.cc user "Krodal".
// (A simple sketch for the motor shield without using the Adafruit library)

// Connector usage
// ---------------
// The order is different than what you would expect.
// If the Arduino (Uno) board is held with the USB
// connector to the left, the positive (A) side is
// at the top (north), and the negative (B) side is
// the bottom (south) for both headers.
//
//   Connector X1:
//     M1 on outside = MOTOR1_A   (+) north
//     M1 on inside  = MOTOR1_B   (-)
//     middle        = GND
//     M2 on inside  = MOTOR2_A   (+)
//     M2 on outside = MOTOR2_B   (-) south
//
//   Connector X2:
//     M3 on outside = MOTOR3_B   (-) south
//     M3 on inside  = MOTOR3_A   (+)
//     middle        = GND
//     M4 on inside  = MOTOR4_B   (-)
//     M4 on outside = MOTOR4_A   (+) north
//
//
//         -------------------------------
//         | -+s                         |
//         | -+s                         |
//    M1 A |                             | M4 A
//    M1 B |                             | M4 B
//    GND  |                             | GND
//    M2 A |                             | M3 A
//    M2 B |                             | M3 B
//         |                       ..... |
//         -------------------------------
//                + -
//
//
//
// Pin usage with the Motorshield
// ---------------------------------------
// Analog pins: not used at all
//     A0 ... A5 are still available
//     They all can also be used as digital pins.
//     Also I2C (A4=SDA and A5=SCL) can be used.
//     These pins have a breadboard area on the shield.
// Digital pins: used: 3,4,5,6,7,8,9,10,11,12
//     Pin 9 and 10 are only used for the servo motors.
//     Already in use: 0 (RX) and 1 (TX).
//     Pin 2 has an soldering hole on the board,?? (I dont see it)
//           easy to connect a wire -> used for the led control
//     Unused: 13
//     Pin 13 is also connected to the system led.
// I2C is possible, but SPI is not possible since
// those pins are used.
//

//Used librarys
#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <FastLED.h>

// Arduino pins for the shift register
#define MOTORLATCH 12
#define MOTORCLK 4
#define MOTORENABLE 7
#define MOTORDATA 8

// 8-bit bus after the 74HC595 shift register
// (not Arduino pins)
// These are used to set the direction of the bridge driver.
#define MOTOR1_A 2
#define MOTOR1_B 3
#define MOTOR2_A 1
#define MOTOR2_B 4
#define MOTOR3_A 5
#define MOTOR3_B 7
#define MOTOR4_A 0
#define MOTOR4_B 6

// Arduino pins for the PWM signals.
#define MOTOR1_PWM 11
#define MOTOR2_PWM 3
#define MOTOR3_PWM 6
#define MOTOR4_PWM 5
#define SERVO1_PWM 10
#define SERVO2_PWM 9

//Arduino pins and variables for the LEDs
#define LED_PIN     2
//#define NUM_LEDS    38
//#define BRIGHTNESS  64
//CRGB leds[NUM_LEDS];
#define NUM_LEDS    24
#define BRIGHTNESS  0.7 // 1 = 100% = max brightness
#define OFFSET      50
#define LED_TYPE    WS2811
#define COLOR_ORDER GRB
CRGB leds[NUM_LEDS];


//For signaling
long unsigned int ledMillis =0;
int motion =0;
// 0 = no motion
// 1 = stop motion
// 2 = left motion
// 3 = right motion
// 4 = forwards motion
// 5 = backwards motion
float rgb[] = {20,180,20};
float rgb2[] = {0,0,0};


// Codes for the motor function and led lights.
#define FORWARD 1
#define BACKWARD 2
#define BRAKE 3
#define RELEASE 4
#define LEFT 5
#define RIGHT 6




//create the ros node nh. The node will be used to publish to Arduino
ros::NodeHandle nh;


//Initializing the functions;
void check_for_led_motion_execution(float cpg_brightness_1 ,float cpg_brightness_2);
void update_led(float cpg_brightness_1 ,float cpg_brightness_2,  int motion_change_signal);
void motor(int nMotor, int command, int speed);
void motor_output (int output, int high_low, int speed);
void shiftWrite(int output, int high_low);
void stop_motion(long unsigned int x);
void rgb2_flash(long unsigned int x);
void forward_motion(long unsigned int x, float cpg_brightness_1 , float cpg_brightness_2);
void backward_motion(long unsigned int x, float cpg_brightness_1 , float cpg_brightness_2);

//*A simple sequence is run where the valves and motors are switched on and off the power the two actuators

//PUMP1: is connected to M1_A(+) and M1_B(-)
//VALVE1: is connected to M2_A(+) and M2_B(-)
//PUMP2: is connected to M4_A(+) and M4_B(-)
//VALVE2: is connected to M3_A(+) and M3_B(-)
void messageCb(const std_msgs::Float32MultiArray& data){
  
  float MOTOR1 = data.data[0];
  float MOTOR2 = data.data[1];

  if(data.data[5] == 1){
    update_led(OFFSET + data.data[2]*BRIGHTNESS , OFFSET + data.data[3]*BRIGHTNESS, data.data[4]);
  } else {
    for (int i = 0; i <= 24; i++) {
    leds[i] = CRGB (0, 0, 0);
    }
  FastLED.show();
  }
  
  if (MOTOR1 == 1){
    motor(1, FORWARD, 255); //Open motor
    motor_output(MOTOR2_A, LOW, 255); //Close valve
  }
  else if (MOTOR1 == 0){
    motor(1, RELEASE, 0); //Close motor
    motor_output(MOTOR2_A, HIGH, 255);//Open valve
  }
  if (MOTOR2 == 1){
    motor(4, FORWARD, 255); //Open motor
    motor_output(MOTOR3_A, LOW, 255); //Close valve
  }
  else if (MOTOR2 == 0){
    motor(4, RELEASE, 0); //Close motor
    motor_output(MOTOR3_A, HIGH, 255); //Open valve
  }
  
}

ros::Subscriber<std_msgs::Float32MultiArray> sub("arduino_control", &messageCb);

void setup()
{
  Serial.begin(57000);
  nh.getHardware()->setBaud(57000);
  nh.initNode();

  nh.subscribe(sub);

  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection( TypicalLEDStrip );

  for (int i = 0; i <= 4; i++) {
    leds[i] = CRGB (20, 200, 20);
    leds[19+i] = CRGB (20, 255, 20);
  }
  for (int i = 5; i <= 18; i++) {
    leds[i] = CRGB (20, 180, 20);
  }
  FastLED.show();
}

void loop()
{
  nh.spinOnce();
}


void update_led(float cpg_brightness_1 ,float cpg_brightness_2,  int motion_change_signal){


  if(motion == LEFT){
    //Light breathing update
    for (int i = 0; i <= 4; i++) {
      leds[i] = CRGB (rgb2[0], rgb2[1], rgb2[2]);
      leds[19+i] = CRGB ((int) cpg_brightness_1*1.5*(rgb[0]/255), (int) cpg_brightness_1*1.5*(rgb[1]/255), (int) cpg_brightness_1*1.5*(rgb[2]/255));
    }
    for (int i = 0; i <= 6; i++) {
      leds[5 + i] = CRGB (rgb2[0], rgb2[1], rgb2[2]);
      leds[12 + i] = CRGB ((int) cpg_brightness_2*(rgb[0]/255), (int) cpg_brightness_2*(rgb[1]/255), (int) cpg_brightness_2*(rgb[2]/255));
    }
  } else if(motion == RIGHT){
    //Light breathing update
    for (int i = 0; i <= 4; i++) {
      leds[i] = CRGB ((int) cpg_brightness_1*(rgb[0]/255), (int) cpg_brightness_1*(rgb[1]/255), (int) cpg_brightness_1*(rgb[2]/255));
      leds[19+i] = CRGB (rgb2[0], rgb2[1], rgb2[2]);
    }
    for (int i = 0; i <= 6; i++) {
      leds[5 + i] = CRGB ((int) cpg_brightness_2*(rgb[0]/255), (int) cpg_brightness_2*(rgb[1]/255), (int) cpg_brightness_2*(rgb[2]/255));
      leds[12 + i] = CRGB (rgb2[0], rgb2[1], rgb2[2]);
    }
  } else if(motion == FORWARD || motion == BACKWARD){
    
  } else {
    //Light breathing update
    for (int i = 0; i <= 4; i++) {
      leds[i] = CRGB ((int) cpg_brightness_1*1.15*(rgb[0]/255), (int) cpg_brightness_1*1.15*(rgb[1]/255), (int) cpg_brightness_1*1.15*(rgb[2]/255));
      leds[19+i] = CRGB ((int) cpg_brightness_1*1.3*(rgb[0]/255), (int) cpg_brightness_1*1.3*(rgb[1]/255), (int) cpg_brightness_1*1.3*(rgb[2]/255));
    }
    for (int i = 5; i <= 18; i++) {
      leds[i] = CRGB ((int) cpg_brightness_2*(rgb[0]/255), (int) cpg_brightness_2*(rgb[1]/255), (int) cpg_brightness_2*(rgb[2]/255));
    }
  }

  //ledMillis gebruiken voor de witte let te laten lopen

  if(motion != 0){
    check_for_led_motion_execution(cpg_brightness_1 ,cpg_brightness_2); 
  }
  
  if(motion_change_signal != 0){
    motion = motion_change_signal;
    
    ledMillis = millis();
  }

    //Green light breathing update
      // for (int i = 0; i <= 4; i++) {
      //   leds[i] = CRGB (0, cpg_brightness_1, 0);
      //   leds[19+i] = CRGB (0, cpg_brightness_1*1.4, 0);
      // }
      // for (int i = 5; i <= 18; i++) {
      //   leds[i] = CRGB (0, cpg_brightness_2, 0);
      // }
    //Red light breathing update
      // for (int i = 0; i <= 4; i++) {
      //   leds[i] = CRGB (cpg_brightness_1, 0, 0);
      //   leds[19+i] = CRGB (cpg_brightness_1*1.4, 0, 0);
      // }
      // for (int i = 5; i <= 18; i++) {
      //   leds[i] = CRGB (cpg_brightness_2, 0, 0);
      // }
    //Blue light breathing update
      // for (int i = 0; i <= 4; i++) {
      //   leds[i] = CRGB (0, 0, cpg_brightness_1);
      //   leds[19+i] = CRGB (0, 0, cpg_brightness_1*1.4);
      // }
      // for (int i = 5; i <= 18; i++) {
      //   leds[i] = CRGB (0, 0, cpg_brightness_2);
      // }
    //Yellow light breathing update
      // for (int i = 0; i <= 4; i++) {
      //   leds[i] = CRGB (cpg_brightness_1, cpg_brightness_1, 0);
      //   leds[19+i] = CRGB (cpg_brightness_1*1.4, cpg_brightness_1*1.4, 0);
      // }
      // for (int i = 5; i <= 18; i++) {
      //   leds[i] = CRGB (cpg_brightness_2, cpg_brightness_2, 0);
      // }
    //No lights breathing update
      // for (int i = 0; i <= 4; i++) {
      //   leds[i] = CRGB (0,0,0);
      //   leds[19+i] = CRGB (0,0,0);
      // }
      // for (int i = 5; i <= 18; i++) {
      //   leds[i] = CRGB (0,0,0);
      // }
 
  FastLED.show();
}

void check_for_led_motion_execution(float cpg_brightness_1 ,float cpg_brightness_2){
  long unsigned int x;
  switch(motion){
    case BRAKE:
      x = millis() - ledMillis;
      if(x >100){
      stop_motion(x-100);
      }
    break; 
    case LEFT:
      x = millis() - ledMillis;
      if(x >100){
        rgb2_flash(x-100);
      } else {
        rgb[0] = 20;
        rgb[1] = 255;
        rgb[2] = 20;
      }    
    break;
    case RIGHT:
      x = millis() - ledMillis;
      if(x >100){
        rgb2_flash(x-100);
      } else {
        rgb[0] = 20;
        rgb[1] = 255;
        rgb[2] = 20;
      }
    break;
    case FORWARD:
      x = millis() - ledMillis;

      if(x >100){
      forward_motion((x-100), cpg_brightness_1, cpg_brightness_2); //We gaan hier de CPG brightness aan meegeven
      } else {
        for (int i = 0; i <= 4; i++) {
          leds[i] = CRGB (15, 180, 15);
          leds[19+i] = CRGB (15*1.4, 180*1.4, 15*1.4);
        }
        for (int i = 5; i <= 18; i++) {
          leds[i] = CRGB (15, 180, 15);
        }
      }
    break;
    case BACKWARD:
      x = millis() - ledMillis;

      if(x >100){
      backward_motion((x-100), cpg_brightness_1, cpg_brightness_2); //We gaan hier de CPG brightness aan meegeven
      } else {
        for (int i = 0; i <= 4; i++) {
          leds[i] = CRGB (15, 180, 15);
          leds[19+i] = CRGB (15*1.4, 180*1.4, 15*1.4);
        }
        for (int i = 5; i <= 18; i++) {
          leds[i] = CRGB (15, 180, 15);
        }
      }
    break;

  }
}

//The next functions are the implementation of the light signals, called in check_for_led_motion_execution()
void stop_motion(long unsigned int x){
  //Start with increasing red color
    if(x <= 500){
      rgb[0] = 255*((float)x/500);
      rgb[1] = 20*((float)x/500);
      rgb[2] = 20*((float)x/500);
    }
  //Switch to steady red color
    else if(x > 500 & x <= 4500){
      rgb[0] = 255;
      rgb[1] = 20;
      rgb[2] = 20;
    }
  //Back to green color
    else if(x >= 4500 & x <= 5000 ){
      rgb[0] = 20 + 245*(5000 - x)/500;
      rgb[1] = 20 + 245*((float)(x-4500)/500);
      rgb[2] = 20;
    }  
    else {
      motion = 0;
    } 
}
void rgb2_flash(long unsigned int x){
    x = x%1000;

  //Smooth rise to white
    if(x <= 100){
      rgb2[0] = 10 + 245*((float)x/100);
      rgb2[1] = 255*((float)x/100);
      rgb2[2] = 10 + 245*((float)x/100);
    }
  //Stay White
    if(x > 100 & x <= 700){
      rgb2[0] = 255;
      rgb2[1] = 255;
      rgb2[2] = 255;
    }
    if(x > 700 & x <= 800){
      rgb2[0] = 10 + 245*(800 - x)/100;
      rgb2[1] = 255;
      rgb2[2] = 10 + 245*(800 - x)/100;
    }
    if(x > 800 & x <= 1000){
      rgb2[0] = 10;
      rgb2[1] = 255;
      rgb2[2] = 10;
    }
}

void forward_motion(long unsigned int x, float cpg_brightness_1 ,float cpg_brightness_2){
  int i = x/100;
    leds[13 + (i-3)%11] = CRGB (20*cpg_brightness_1/255, 255*cpg_brightness_1/255, 20*cpg_brightness_1/255);
    leds[13 + (i-2)%11] = CRGB (200*cpg_brightness_1/255, 200*cpg_brightness_1/255, 200*cpg_brightness_1/255);
    leds[13 + (i-1)%11] = CRGB (255*cpg_brightness_1/255, 255*cpg_brightness_1/255, 255*cpg_brightness_1/255);
    leds[13 + i%11] = CRGB (200*cpg_brightness_1/255, 200*cpg_brightness_1/255, 200*cpg_brightness_1/255);

    leds[10 - (i-3)%11] = CRGB (20*cpg_brightness_1/255, 255*cpg_brightness_1/255, 20*cpg_brightness_1/255);
    leds[10 - (i-2)%11] = CRGB (200*cpg_brightness_1/255, 200*cpg_brightness_1/255, 200*cpg_brightness_1/255);
    leds[10 - (i-1)%11] = CRGB (255*cpg_brightness_1/255, 255*cpg_brightness_1/255, 255*cpg_brightness_1/255);  
    leds[10 - i%11] = CRGB (200*cpg_brightness_1/255, 200*cpg_brightness_1/255, 200*cpg_brightness_1/255);

}
void backward_motion(long unsigned int x, float cpg_brightness_1 ,float cpg_brightness_2){
    int i = x/100;
    leds[23 - (i+3)%11] = CRGB (200*cpg_brightness_1/255, 200*cpg_brightness_1/255, 200*cpg_brightness_1/255);
    leds[23 - (i+2)%11] = CRGB (255*cpg_brightness_1/255, 255*cpg_brightness_1/255, 255*cpg_brightness_1/255);
    leds[23 - (i+1)%11] = CRGB (200*cpg_brightness_1/255, 200*cpg_brightness_1/255, 200*cpg_brightness_1/255);
    leds[23 - i%11] = CRGB (20*cpg_brightness_1/255, 255*cpg_brightness_1/255, 20*cpg_brightness_1/255);
    
    leds[12] = CRGB (20*cpg_brightness_1/255, 255*cpg_brightness_1/255, 20*cpg_brightness_1/255);
    leds[11] = CRGB (20*cpg_brightness_1/255, 255*cpg_brightness_1/255, 20*cpg_brightness_1/255);

    leds[i%11] = CRGB (20*cpg_brightness_1/255, 255*cpg_brightness_1/255, 20*cpg_brightness_1/255);
    leds[(i+1)%11] = CRGB (200*cpg_brightness_1/255, 200*cpg_brightness_1/255, 200*cpg_brightness_1/255);
    leds[(i+2)%11] = CRGB (255*cpg_brightness_1/255, 255*cpg_brightness_1/255, 255*cpg_brightness_1/255);   
    leds[(i+3)%11] = CRGB (200*cpg_brightness_1/255, 200*cpg_brightness_1/255, 200*cpg_brightness_1/255);  
};

// Initializing
// ------------
// There is no initialization function.
//
// The shiftWrite() has an automatic initializing.
// The PWM outputs are floating during startup,
// that's okay for the Motor Shield, it stays off.
// Using analogWrite() without pinMode() is valid.
//


// ---------------------------------
// motor
//
// Select the motor (1-4), the command,
// and the speed (0-255).
// The commands are: FORWARD, BACKWARD, BRAKE, RELEASE.

void motor(int nMotor, int command, int speed)
{
  int motorA, motorB;

  if (nMotor >= 1 && nMotor <= 4)
  {  
    switch (nMotor)
    {
    case 1:
      motorA   = MOTOR1_A;
      motorB   = MOTOR1_B;
      break;
    case 2:
      motorA   = MOTOR2_A;
      motorB   = MOTOR2_B;
      break;
    case 3:
      motorA   = MOTOR3_A;
      motorB   = MOTOR3_B;
      break;
    case 4:
      motorA   = MOTOR4_A;
      motorB   = MOTOR4_B;
      break;
    default:
      break;
    }

    switch (command)
    {
    case FORWARD:
      motor_output (motorA, HIGH, speed);
      motor_output (motorB, LOW, -1);     // -1: no PWM set
      break;
    case BACKWARD:
      motor_output (motorA, LOW, speed);
      motor_output (motorB, HIGH, -1);    // -1: no PWM set
      break;
    case BRAKE:
      // The AdaFruit library didn't implement a brake.
      // The L293D motor driver ic doesn't have a good
      // brake anyway.
      // It uses transistors inside, and not mosfets.
      // Some use a software break, by using a short
      // reverse voltage.
      // This brake will try to brake, by enabling
      // the output and by pulling both outputs to ground.
      // But it isn't a good break.
      motor_output (motorA, LOW, 255); // 255: fully on.
      motor_output (motorB, LOW, -1);  // -1: no PWM set
      break;
    case RELEASE:
      motor_output (motorA, LOW, 0);  // 0: output floating.
      motor_output (motorB, LOW, -1); // -1: no PWM set
      break;
    default:
      break;
    }
  }
}


// ---------------------------------
// motor_output
//
// The function motor_ouput uses the motor driver to
// drive normal outputs like lights, relays, solenoids,
// DC motors (but not in reverse).
//
// It is also used as an internal helper function
// for the motor() function.
//
// The high_low variable should be set 'HIGH'
// to drive lights, etc.
// It can be set 'LOW', to switch it off,
// but also a 'speed' of 0 will switch it off.
//
// The 'speed' sets the PWM for 0...255, and is for
// both pins of the motor output.
//   For example, if motor 3 side 'A' is used to for a
//   dimmed light at 50% (speed is 128), also the
//   motor 3 side 'B' output will be dimmed for 50%.
// Set to 0 for completelty off (high impedance).
// Set to 255 for fully on.
// Special settings for the PWM speed:
//    Set to -1 for not setting the PWM at all.
//
void motor_output (int output, int high_low, int speed)
{
  int motorPWM;

  switch (output)
  {
  case MOTOR1_A:
  case MOTOR1_B:
    motorPWM = MOTOR1_PWM;
    break;
  case MOTOR2_A:
  case MOTOR2_B:
    motorPWM = MOTOR2_PWM;
    break;
  case MOTOR3_A:
  case MOTOR3_B:
    motorPWM = MOTOR3_PWM;
    break;
  case MOTOR4_A:
  case MOTOR4_B:
    motorPWM = MOTOR4_PWM;
    break;
  default:
    // Use speed as error flag, -3333 = invalid output.
    speed = -3333;
    break;
  }

  if (speed != -3333)
  {
    // Set the direction with the shift register
    // on the MotorShield, even if the speed = -1.
    // In that case the direction will be set, but
    // not the PWM.
    shiftWrite(output, high_low);

    // set PWM only if it is valid
    if (speed >= 0 && speed <= 255)    
    {
      analogWrite(motorPWM, speed);
    }
  }
}


// ---------------------------------
// shiftWrite
//
// The parameters are just like digitalWrite().
//
// The output is the pin 0...7 (the pin behind
// the shift register).
// The second parameter is HIGH or LOW.
//
// There is no initialization function.
// Initialization is automatically done at the first
// time it is used.
//
void shiftWrite(int output, int high_low)
{
  static int latch_copy;
  static int shift_register_initialized = false;

  // Do the initialization on the fly,
  // at the first time it is used.
  if (!shift_register_initialized)
  {
    // Set pins for shift register to output
    pinMode(MOTORLATCH, OUTPUT);
    pinMode(MOTORENABLE, OUTPUT);
    pinMode(MOTORDATA, OUTPUT);
    pinMode(MOTORCLK, OUTPUT);

    // Set pins for shift register to default value (low);
    digitalWrite(MOTORDATA, LOW);
    digitalWrite(MOTORLATCH, LOW);
    digitalWrite(MOTORCLK, LOW);
    // Enable the shift register, set Enable pin Low.
    digitalWrite(MOTORENABLE, LOW);

    // start with all outputs (of the shift register) low
    latch_copy = 0;

    shift_register_initialized = true;
  }

  // The defines HIGH and LOW are 1 and 0.
  // So this is valid.
  bitWrite(latch_copy, output, high_low);

  // Use the default Arduino 'shiftOut()' function to
  // shift the bits with the MOTORCLK as clock pulse.
  // The 74HC595 shiftregister wants the MSB first.
  // After that, generate a latch pulse with MOTORLATCH.
  shiftOut(MOTORDATA, MOTORCLK, MSBFIRST, latch_copy);
  delayMicroseconds(5);    // For safety, not really needed.
  digitalWrite(MOTORLATCH, HIGH);
  delayMicroseconds(5);    // For safety, not really needed.
  digitalWrite(MOTORLATCH, LOW);
}
