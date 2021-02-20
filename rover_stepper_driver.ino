/* Rover Stepper Motor Control for use with ROS - Kelvin Cui */

#include <AccelStepper.h>

#define right_motorPin1  2      
#define right_motorPin2  3      
#define right_motorPin3  4     
#define right_motorPin4  5     

#define left_motorPin1 6
#define left_motorPin2 7
#define left_motorPin3 8
#define left_motorPin4 9



#define MotorInterfaceType 8

AccelStepper right_stepper = AccelStepper(MotorInterfaceType, right_motorPin1, right_motorPin3, right_motorPin2, right_motorPin4);
AccelStepper left_stepper = AccelStepper(MotorInterfaceType, left_motorPin1, left_motorPin3, left_motorPin2, left_motorPin4);

const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];
char msgChar[numChars] = {0};

float right_speed, right_cmd, left_speed, left_cmd;

boolean newData = false;
int sleeptimer;

void setup() {
  // Set the maximum steps per second:
  right_stepper.setMaxSpeed(1100);
  left_stepper.setMaxSpeed(1100);
  Serial.begin(9600);
  
  right_speed = 0;
  left_speed = 0;
  right_cmd = 0;
  left_cmd = 0;
  sleeptimer = 0;
}
void loop() {
  getData();
  if (newData == true) {
    sleeptimer = 0;
    right_stepper.enableOutputs();
    left_stepper.enableOutputs();
    strcpy(tempChars, receivedChars);
    parseData();
    newData = false;
  }

  //28byj is 4076 steps for 1 revolution - therefore 11.3 steps per degree or 648.7step/rad
  right_cmd = (right_speed * -648.7);
  left_cmd = (left_speed * 648.7);
  
  if (sleeptimer > 25000){
    //Serial.print("Sleeping | ");
    right_stepper.setSpeed(0);
    left_stepper.setSpeed(0);

    right_stepper.disableOutputs();
    left_stepper.disableOutputs();
  }

  else {
    right_stepper.setSpeed(right_cmd);
    left_stepper.setSpeed(left_cmd);

    sleeptimer += 1;
  }
  right_stepper.runSpeed();
  left_stepper.runSpeed();
  //Serial.println(sleeptimer);
}

void getData() {
  static boolean inprogress = false;
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;

  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();

    if (inprogress == true) {
      if (rc != endMarker) {
        receivedChars[ndx] = rc;
        ndx += 1;
        if (ndx >= numChars) {
          ndx = numChars - 1;
        }
      }

      else{
        receivedChars[ndx] = '\0';
        inprogress = false;
        ndx = 0;
        newData = true;
      }
    }

    else if (rc == startMarker) {
      inprogress = true;
    }
  }
}

void parseData() {
  char * strtokndx;

  strtokndx = strtok(tempChars, ",");
  right_speed = atof(strtokndx);

  strtokndx = strtok(NULL, ",");
  left_speed = atof(strtokndx);
}
