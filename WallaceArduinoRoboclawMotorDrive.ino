#include <SoftwareSerial.h>
#include "RoboClaw.h"

#define rcRxPin 10
#define rcTxPin 11
#define address 0x80
#define C2F_FACTOR 9/5
#define C2F(x) (x * C2F_FACTOR) + 32

#define MAX_USB_RX_BUF 64
#define MAX_CMD_BUF 3
#define MAX_PARM_BUF 4

char receivedChars[MAX_USB_RX_BUF];   // an array to store the received data
bool newData = false;

bool usbDataReadyToParse = false;

char command[MAX_CMD_BUF] = {'\0'};
char param1[MAX_PARM_BUF]  = {'\0'};
char param2[MAX_PARM_BUF]  = {'\0'};

bool newCommandIsReady = false;

long numCmdsRxdFromUSB = 0;

bool rcValid;
uint8_t rcStatus;

char version[32];
uint16_t volts;
int16_t amps1, amps2;
uint16_t temp;
int32_t speedM1, speedM2;

unsigned long prevMillisLastTimeReadVoltsAmpsTemp = millis();
unsigned long prevMillisLastCommand = millis();
unsigned long nowMillis = millis();

bool motorM1Rotating = false;
bool motorM2Rotating = false;


SoftwareSerial serial(rcRxPin, rcTxPin);
RoboClaw roboclaw(&serial, 10000);

void setup() {
  //Open Serial and roboclaw serial ports
  Serial.begin(115200);
  roboclaw.begin(38400);
}

///////////////////////////////////////////////////////////////////////////////////////
//  MAIN LOOP
///////////////////////////////////////////////////////////////////////////////////////
void loop() {

  stopMotorsIfBeenTooLongSinceLastCommand();
  readRcVoltsAmpsTempIfBeenTooLongSinceLastTime();
  
  recvIncomingUsbSerialWithEndMarker();
  showIncomingUsbSerial(false);
  parseIncomingUsbSerial(false);
  commandHandler();

  delay(10);
}







void commandHandler() {

  if (!newCommandIsReady) return;

  prevMillisLastCommand = millis();
  numCmdsRxdFromUSB++;

  long int cmd = strtol(command, NULL, 10);
  
  switch (cmd) {

    //////////////////// Arduino user - instructions /////////////////////////
    case 0: //help - send list of available commands
      getHelp();
      break;
    case 1: //num cmds rxd from USB
      getNumCmdsRxdFromUSB();
      break;

    //////////////////// Roboclaw instructions /////////////////////////
    case 20: //version
      readRcVersion();
      break;
    case 21: //voltage
      readRcVoltage(true);
      break;
    case 22: //amps
      readRcCurrents(true);
      break;
    case 23: //temperature
      readRcTemperature(true);
      break;
    case 24: //read volts, amps, temp
      readRcVoltsAmpsTemp();
      break;
    case 25: //read both motors speed
      readRcMotorSpeeds(true);
      break;
    case 26: //move motor 1 forward
      rotateLeftSideForward(param1, true);
      break;
    case 27: //move motor 2 forward
      rotateRightSideForward(param1, true);
      break;
    case 28: //stop both motors
      stopMotors(true);
      break;
    case 29:
      moveForward(true);
      break;
    case 30:
      rotateLeftSideBackward(param1, true);
      break;
    case 31:
      rotateRightSideBackward(param1, true);
      break;
    case 32:
      moveBackward(true);
      break;
    case 33:
      rotateLeft(true);
      break;
    case 34:
      rotateRight(true);
      break;
    default:
      Serial.println("Err. Choose one:");
      getHelp();
  }

  newCommandIsReady = false;
}

//////////////////// Arduino user - instructions /////////////////////////
//////////////////// Arduino user - instructions /////////////////////////
//////////////////// Arduino user - instructions /////////////////////////
void getHelp() {
  Serial.println("20 - version");
  Serial.println("21 - volts");
  Serial.println("22 - amps");
  Serial.println("23 - temp");
  Serial.println("24 - volts,amps,temp");
  Serial.println("25 - read speeds");
  Serial.println("26 - rot M1 fwd");
  Serial.println("27 - rot M2 fwd");
  Serial.println("28 - stop motors");
  Serial.println("29 - move forward");
  Serial.println("30 - rot M1 bck");
  Serial.println("31 - rot M2 bck");
  Serial.println("32 - move back");
  Serial.println("33 - rot left");
  Serial.println("34 - rot right");
  Serial.println("0 - help");
  Serial.println("1 - num cmds rxd");
}

void getNumCmdsRxdFromUSB() {
  Serial.println(numCmdsRxdFromUSB);
}





//////////////////// Roboclaw instructions /////////////////////////
//////////////////// Roboclaw instructions /////////////////////////
//////////////////// Roboclaw instructions /////////////////////////
void readRcVersion() {
  for (byte i = 0; i < sizeof(version); i++) {
    version[i] = 0;
  }
  if (roboclaw.ReadVersion(address, version)) {
    Serial.println(version);
  } else {
    Serial.println("ERVER");
  }
}


/*
   Read Main Battery Voltage Level
   Read the main battery voltage level connected to B+ and B- terminals.
   The voltage is returned in 10ths of a volt(eg 300 = 30v).
*/
void readRcVoltage(bool print) {
  if (print) Serial.println("reading voltage");

  rcValid = false;
  volts = roboclaw.ReadMainBatteryVoltage(address, &rcValid);
  if (rcValid) {
    if (print) Serial.println(volts / 10.0f);
  } else {
    if (print) Serial.println("ERVOLT");
    else volts = -1;
  }
}

/*
    Read Motor Currents
   Read the current draw from each motor in 10ma increments.
   The amps value is calculated by dividing the value by 100.
*/
void readRcCurrents(bool print) {
  if (print) Serial.println("reading currents");

  if (roboclaw.ReadCurrents(address, amps1, amps2)) {
    if (print) {
      Serial.print(amps1 / 100.0f);
      Serial.print(" , ");
      Serial.println(amps2 / 100.0f);
    }
  } else {
    if (print) Serial.println("ERAMPS");
    else {
      amps1 = -1;
      amps2 = -1;
    }
  }

}

/*
   Read Temperature
   Read the board temperature. Value returned is in 10ths of degrees.
*/
void readRcTemperature(bool print) {
  if (print) Serial.println("reading temp");

  if (roboclaw.ReadTemp(address, temp)) {
    if (print) Serial.println(C2F(temp / 10.0f));
  } else {
    if (print) Serial.println("ERTEMP");
    else temp = -1;
  }
}

void readRcVoltsAmpsTemp() {
  readRcVoltage(false);
  readRcCurrents(false);
  readRcTemperature(false);
  String results = "{'v':";
  results.concat(volts / 10.0f);
  results.concat(",'a1':");
  results.concat(amps1 / 100.0f);
  results.concat(",'a2':");
  results.concat(amps2 / 100.0f);
  results.concat(",'t':");
  results.concat(C2F(temp / 10.0f));
  results.concat("}");
  Serial.println(results);
}

void readRcMotorSpeeds(bool print) {

  if (print) Serial.println("reading motor speeds");

  rcStatus = 0; rcValid = false;
  speedM1 = roboclaw.ReadSpeedM1(address, &rcStatus, &rcValid);
  if (!rcValid) {
    if (print) { Serial.print("ERRDSPM1 : "); Serial.print(rcStatus, HEX); Serial.print(" "); Serial.println(rcValid); }
    speedM1 = -1;
  }

  rcStatus = 0; rcValid = false;
  speedM2 = roboclaw.ReadSpeedM2(address, &rcStatus, &rcValid);
  if (!rcValid) {
    if (print) { Serial.print("ERRDSPM2 : "); Serial.print(rcStatus, HEX); Serial.print(" "); Serial.println(rcValid); }
    speedM2 = -1;
  }

  String results = "{'m1':";
  results.concat(speedM1);
  results.concat(",'m2':");
  results.concat(speedM2);
  results.concat("}");
  Serial.println(results);
}

void rotateLeftSideForward(char* param, bool print) {
  uint8_t speed = (uint8_t)strtol(param, NULL, 10);
  if (roboclaw.ForwardM1(address, speed)) {
    motorM1Rotating = true;
    readRcMotorSpeeds(print);
  } else {
    if (print) Serial.println("ERRROTM1F");
  }
}

void rotateRightSideForward(char* param, bool print) {
  uint8_t speed = (uint8_t)strtol(param, NULL, 10);
  if (roboclaw.ForwardM2(address, speed)) {
    motorM2Rotating = true;
    readRcMotorSpeeds(print);
  } else {
    if (print) Serial.println("ERRROTM2F");
  }
}

void stopMotors(bool print) {
  uint8_t speed = 0;
  if (roboclaw.ForwardM1(address, speed)) {
    if (print) Serial.println("M1 stopped");
    motorM1Rotating = false;
  } else {
    if (print) Serial.println("ERRSTOPM1");
  }
  if (roboclaw.ForwardM2(address, speed)) {
    if (print) Serial.println("M2 stopped");
    motorM2Rotating = false;
  } else {
    if (print) Serial.println("ERRROTM1F");
  }
}

void moveForward(bool print) {
  rotateLeftSideForward(param1, print);
  rotateRightSideForward(param2, print);
}

void rotateLeftSideBackward(char* param, bool print) {
  uint8_t speed = (uint8_t)strtol(param, NULL, 10);
  if (roboclaw.BackwardM1(address, speed)) {
    motorM1Rotating = true;
    readRcMotorSpeeds(print);
  } else {
    if (print) Serial.println("ERRROTM1B");
  }
}

void rotateRightSideBackward(char* param, bool print) {
  uint8_t speed = (uint8_t)strtol(param, NULL, 10);
  if (roboclaw.BackwardM2(address, speed)) {
    motorM2Rotating = true;
    readRcMotorSpeeds(print);
  } else {
    if (print) Serial.println("ERRROTM2B");
  }
}

void moveBackward(bool print) {
  rotateLeftSideBackward(param1, print);
  rotateRightSideBackward(param2, print);
}

void rotateLeft(bool print) {
  rotateLeftSideBackward(param1, print);
  rotateRightSideForward(param2, print);
}

void rotateRight(bool print) {
  rotateLeftSideForward(param1, print);
  rotateRightSideBackward(param2, print);
}


//////////////////// private functions /////////////////////////
//////////////////// private functions /////////////////////////
//////////////////// private functions /////////////////////////

void recvIncomingUsbSerialWithEndMarker() {
  static byte ndx = 0;
  char endMarker = '\n';
  char rc;

  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();

    if (rc != endMarker) {
      receivedChars[ndx] = rc;
      ndx++;
      if (ndx >= MAX_USB_RX_BUF) {
        ndx = MAX_USB_RX_BUF - 1;
      }
    }
    else {
      receivedChars[ndx] = '\0'; // terminate the string
      ndx = 0;
      newData = true;
    }
  }

}

void showIncomingUsbSerial(bool print) {
  if (newData == true) {
    if (receivedChars[0] != '\n' && receivedChars[0] != ' ' && receivedChars[0] != '\t' && receivedChars[0] != '\r' && receivedChars[0] != 0) {
      if (print) { Serial.print("This just in ... ["); Serial.print(receivedChars); Serial.println("]"); }
      usbDataReadyToParse = true;
    }
    newData = false;
  }
}

void parseIncomingUsbSerial(bool print) {

  if (usbDataReadyToParse == true) {
    clearBuffer(command, MAX_CMD_BUF);
    clearBuffer(param1, MAX_PARM_BUF);
    clearBuffer(param2, MAX_PARM_BUF);
    Serial.println("..parsing..");
    
    byte ndx = 0;

    /////   extract  command out of the receive buffer
    while (ndx < MAX_USB_RX_BUF && ndx < MAX_CMD_BUF - 1 && receivedChars[ndx] != '\n' && receivedChars[ndx] != ' ' && receivedChars[ndx] != '\t' && receivedChars[ndx] != '\r' && receivedChars[ndx] != 0) {
      //Serial.print(ndx); Serial.print(", ["); Serial.print(receivedChars[ndx]); Serial.println("]");
      command[ndx] = receivedChars[ndx];
      ndx++;
    }
    if (print) {
      Serial.print("command[");
      Serial.print(command);
      Serial.print("] ");
      Serial.println(ndx);
    }

    /////   need to get past whitespace (except newline)
    byte sidx = 0;
    while (ndx < MAX_USB_RX_BUF && (receivedChars[ndx] == ' ' || receivedChars[ndx] == '\t')) {
      //Serial.print(ndx); Serial.print(", ["); Serial.print(receivedChars[ndx]); Serial.println("]");
      ndx++;
    }

    /////   extract param1 out of the receive buffer
    byte pidx = 0;
    while (ndx < MAX_USB_RX_BUF && pidx < MAX_PARM_BUF - 1 && receivedChars[ndx] != '\n' && receivedChars[ndx] != ' ' && receivedChars[ndx] != '\t' && receivedChars[ndx] != '\r' && receivedChars[ndx] != 0) {
      //Serial.print(ndx); Serial.print(", ["); Serial.print(receivedChars[ndx]); Serial.println("]");
      param1[pidx] = receivedChars[ndx];
      ndx++;
      pidx++;
    }
    if (print) {
      Serial.print("param1[");
      Serial.print(param1);
      Serial.print("] ");
      Serial.print(pidx);
      Serial.print(" ");
      Serial.println(ndx);
    }

    /////   need to get past whitespace (except newline)
    sidx = 0;
    while (ndx < MAX_USB_RX_BUF && (receivedChars[ndx] == ' ' || receivedChars[ndx] == '\t')) {
      //Serial.print(ndx); Serial.print(", ["); Serial.print(receivedChars[ndx]); Serial.println("]");
      ndx++;
    }

    /////   extract param2 out of the receive buffer
    pidx = 0;
    while (ndx < MAX_USB_RX_BUF && pidx < MAX_PARM_BUF - 1 && receivedChars[ndx] != '\n' && receivedChars[ndx] != ' ' && receivedChars[ndx] != '\t' && receivedChars[ndx] != '\r' && receivedChars[ndx] != 0) {
      //Serial.print(ndx); Serial.print(", ["); Serial.print(receivedChars[ndx]); Serial.println("]");
      param2[pidx] = receivedChars[ndx];
      ndx++;
      pidx++;
    }
    if (print) {
      Serial.print("param2[");
      Serial.print(param2);
      Serial.print("] ");
      Serial.print(pidx);
      Serial.print(" ");
      Serial.println(ndx);
    }

    usbDataReadyToParse = false;
    clearBuffer(receivedChars, MAX_USB_RX_BUF);
    newCommandIsReady = true;
  }

}


void stopMotorsIfBeenTooLongSinceLastCommand() {
  if (motorM1Rotating || motorM2Rotating) {
    nowMillis = millis();
    if (nowMillis - prevMillisLastCommand > 5000) {
      stopMotors(true);
    }
  }
}

void readRcVoltsAmpsTempIfBeenTooLongSinceLastTime() {
    nowMillis = millis();
    if (nowMillis - prevMillisLastTimeReadVoltsAmpsTemp > 10000) {
      prevMillisLastTimeReadVoltsAmpsTemp = millis();
      readRcVoltsAmpsTemp();      
    }
}

void clearBuffer(char* pBuf, byte len) {
  for (byte i = 0; i < len; i++) {
    pBuf[i] = '\0';
  }
}
