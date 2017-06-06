// Control code for two-channel electrophoresis supply using Volteq HY30005EP supply and
//  external relay box.

// except as otherwise noted,
// Copyright (c) C. Harrison 2017. All rights reserved. BSD 2-clause license

// Portions Copyright (c) 2017 www.volteq.com.  All rights reserved.
// Volteq licenses to you the right to use, modify, and copy
// only for use in a Volteq product.

// external 15-pin interface connector
//   pin 1  - MOSI
//       2  - MISO
//       3  - SS
//       4  - SCK
//       5  - 
//       6  - +5V
//       7  - +12V
//       8  - GND (Main supply output negative)
//       9  - D4  (polarity relay in sample firmware)
//      10  - IO13
//      11  - IO12/A11
//      12  - D5
//      13  - D6/A7
//      14  - RX
//      15  - TX


#include <EEPROM.h>
#include <Wire.h>
#include <LCD.h>
#include <LiquidCrystal_I2C.h>

// LCD display using PCF8574 IO extender; pin assignments on extender chip defined here
#define I2C_ADDR  0x27 // <<- Add your address here.  0x3F for test unit; 0x27 for production
#define Rs_pin 0
#define Rw_pin 1
#define En_pin 2
#define BACKLIGHT_PIN 3
#define D4_pin 4
#define D5_pin 5
#define D6_pin 6
#define D7_pin 7

#define RS485Transmit    LOW
#define RS485Receive     HIGH
#define RS485Pin 8
LiquidCrystal_I2C lcd(I2C_ADDR, En_pin, Rw_pin, Rs_pin, D4_pin, D5_pin, D6_pin, D7_pin);

typedef enum {AmpCmd, VoltCmd, OVoltCmd, EnableOut, NUM_OUT_PINS} OutPinType;
const uint8_t outputPin[NUM_OUT_PINS] = {10, 9, 11, 7};  //pin 10 for I, pin 9 for V, Pin 11 for OVP, pin 7 for Enable/Disable

/* Configure digital pins 9 and 10 as 16-bit PWM OutputStatuss. */
void setupPWM16() {
  DDRB |= _BV(PB1) | _BV(PB2);        /* set pins as outputs */
  TCCR1A = _BV(COM1A1) | _BV(COM1B1)  /* non-inverting PWM */
           | _BV(WGM11);                   /* mode 14: fast PWM, TOP=ICR1 */
  TCCR1B = _BV(WGM13) | _BV(WGM12)
           | _BV(CS10);                    /* no prescaling */
  ICR1 = 0xffff;                      /* TOP counter value */

}

/* 16-bit version of analogWrite(). Works only on pins 9 and 10. */

void analogWrite16(uint8_t pin, uint16_t val) //Control function for PWM 9 & 10
{
  switch (pin) {
    case  9: OCR1A = val; break;
    case 10: OCR1B = val; break;
  }
}

enum UiScreen {MainScreen, SetupScreen, ParamScreen} uiScreen;

enum {OffMode, StartMode, WaitNSMode, WaitEWMode, NSMode, EWMode, TOTAL_MODES} cmailMode;
char modeCode[TOTAL_MODES][3] = {"--", "--", "..", "..", "NS", "EW"};

// Analog input & processing
const unsigned long analogReadInterval = 50; // millisec between analog updates
typedef enum  { NoProc, PotProc, Discrete0, Discrete1, Linearize0, Linearize1 } ProcessType;
typedef enum              {PotChan=0, KeysChan, StatusChan, VoltsChan, AmpsChan, RemoteChan, ANALOG_CHANS} AnalogChan;
const uint8_t analogPin[] = { A0,      A5,        A4,        A2,         A1,       A3 };
const float   smoothingK[]= { 0.5,     0.5,       0.5,       0.1,        0.1,      0.1};
const int8_t  processVal[]= { PotProc,Discrete0,Discrete1,Linearize0,Linearize1,Linearize0};
float    analogSmoothed[ANALOG_CHANS];
float    analogProcessed[ANALOG_CHANS];

const int numDiscretes = 2; // Keys, Status
int discreteValue[numDiscretes];


// CMAIL parameters
float cmailV1, cmailV2;
unsigned int cmailT1, cmailT2, cmailCycles;
// temporary CMAIL parametersused in adjustment screen
float cmailV1_tmp, cmailV2_tmp;
unsigned int cmailT1_tmp, cmailT2_tmp, cmailCycles_tmp;
// CMAIL running values
int cmailCycleNow, cmailSecRemain;
bool cmailPause;

typedef enum {None, CyclesSel, V1Sel, T1Sel, V2Sel, T2Sel} SetupScreenSelector;
SetupScreenSelector setupScreenSelector;

float setpointVolts;

void setup()
{

  pinMode(outputPin[OVoltCmd], OUTPUT);  //PWM OV Control Pin
  analogWrite(outputPin[OVoltCmd], 250); // set OV to maximum
  setupPWM16();
  pinMode(outputPin[VoltCmd], OUTPUT);//PWM Voltage Control Pin
  pinMode(outputPin[AmpCmd], OUTPUT);//PWM Current Control Pin
  analogWrite16(outputPin[VoltCmd], 0);
  analogWrite16(outputPin[AmpCmd], 0);
  pinMode(outputPin[EnableOut], OUTPUT);//Output Enable/Disable
  digitalWrite(outputPin[EnableOut], LOW);//disable Output

  delay(300);
  pinMode(RS485Pin, OUTPUT); //RS485 send/receive pin
  digitalWrite(RS485Pin, RS485Receive);//Set RS485 to listen
 
  analogReference(EXTERNAL);//choose internal 1.1V reference voltage;0:external;1:internal VCC;2:null;3:internal 1.1V
  delay(300);
  lcd.begin (20, 4); // <<-- our LCD is a 20x4, change for your LCD if needed
  // LCD Backlight ON
  lcd.setBacklightPin(BACKLIGHT_PIN, POSITIVE);
  lcd.setBacklight(HIGH);
  delay(300);

  Serial.begin(19200);
  Serial1.begin(9600);
  delay(200);

  cmailMode = OffMode;
  uiScreen = MainScreen;

}


//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
void loop()
{

  readAnalogInputs();
  
  if (Serial.available() > 0) {
    handleSerialInput();
  }
  handleKeyPress();
  handlePanelKnob();
  updateCMAIL();

  updateOutputs();
  updateLCD();

}

void updateOutputs() {
  digitalWrite(outputPin[EnableOut], (cmailMode==OffMode) ? LOW : HIGH);
  const float voltsMax = 300., voltsMaxPWM = 65000.;
  float voltPWM = setpointVolts/voltsMax * voltsMaxPWM;
  analogWrite16(outputPin[VoltCmd], (uint16_t)voltPWM);
  analogWrite(outputPin[OVoltCmd], 250);
  const float ampsMax = 5., ampsMaxPWM = 65000., ampsZeroPWM = 3000.;
  float ampPWM = 0.3/ampsMax * ampsMaxPWM + ampsZeroPWM;
  analogWrite16(outputPin[AmpCmd], (uint16_t)ampPWM);
}

void updateLCD() {
  // full screen update requires 80 msec over I2C; break up into 4 lines 20 msec each pass
  static uint8_t line=0;
  char lcdLine[21];
  if (uiScreen == MainScreen) {
    char voltString[6], ampString[5];
    lcd.setCursor (0, line); // go to start of line
    switch (line) {
      case 0:
        dtostrf(analogSmoothed[VoltsChan]/3.28, 5, 1, voltString);
        dtostrf(analogProcessed[AmpsChan], 4, 2, ampString);
        sprintf(lcdLine, " %s V %s A [%s]", voltString, ampString, modeCode[cmailMode]);
        break;
      case 1:
        sprintf(lcdLine, " #%4d/%-4d %4d sec", cmailCycleNow, cmailCycles, abs(cmailSecRemain));
        break;
      case 2:
        sprintf(lcdLine, "--------------------");
        break;
      case 3:
        sprintf(lcdLine, "SETUP      RST %s", (cmailMode==OffMode || cmailPause) ? " RUN " : "PAUSE");
    }
    line = (line+1)%4;
    lcd.print(lcdLine);
  };
  if (uiScreen == SetupScreen) {
    char voltString[6];
    const char parens[2] = {'(', ')'}, spaces[2] = {' ', ' '};
    const char *sel1, *sel2;
    lcd.setCursor (0, line); // go to start of line
    switch (line) {
      case 0:
        sel1 = (setupScreenSelector==CyclesSel) ? parens : spaces;
        sprintf(lcdLine, "  %c%4d%c Cycles [%s]", sel1[0], cmailCycles_tmp, sel1[1], modeCode[cmailMode]);
        break;
     case 1:
        dtostrf(cmailV1_tmp, 5, 1, voltString);
        sel1 = (setupScreenSelector==V1Sel) ? parens : spaces;
        sel2  = (setupScreenSelector==T1Sel) ? parens : spaces;
        sprintf(lcdLine, "NS%c%s%cV %c%4d%csec",sel1[0], voltString, sel1[1], sel2[0], cmailT1_tmp, sel2[1]);
        break;
     case 2:
        dtostrf(cmailV2_tmp, 5, 1, voltString);
        sel1 = (setupScreenSelector==V2Sel) ? parens : spaces;
        sel2 = (setupScreenSelector==T2Sel) ? parens : spaces;
        sprintf(lcdLine, "EW%c%s%cV %c%4d%csec",sel1[0], voltString, sel1[1], sel2[0], cmailT2_tmp, sel2[1]);
        break;
     case 3:
        sprintf(lcdLine, " SEL  ADJ  OK  CANCL");
    }     
    line = (line+1)%4;
    lcd.print(lcdLine);
  };
}
  
void handlePanelKnob() {
  if (setupScreenSelector==CyclesSel) {
    cmailCycles_tmp = int(9999.0*analogProcessed[PotChan]);
  }
  if (setupScreenSelector==V1Sel) {
    cmailV1_tmp = 250.0*analogProcessed[PotChan];
  }
  if (setupScreenSelector==T1Sel) {
    cmailT1_tmp = int(9999.0*analogProcessed[PotChan]);
  }
  if (setupScreenSelector==V2Sel) {
    cmailV2_tmp = 250.0*analogProcessed[PotChan];
  }
  if (setupScreenSelector==T2Sel) {
    cmailT2_tmp = int(9999.0*analogProcessed[PotChan]);
  }
};


void handleKeyPress() {
  static int lastKey;
  typedef enum {F4Key=10, F3Key, F2Key, F1Key} KeyName;
  int key = discreteValue[0];
  if (lastKey==-1 && key!=-1) {
    // key down
    if (uiScreen==MainScreen) {
      const KeyName SETUP = F1Key;
      const KeyName RST = F3Key;
      const KeyName RUN_PAUSE = F4Key;
      if (key == SETUP) {
        uiScreen = SetupScreen;
        setupScreenSelector = None;
        cmailCycles_tmp = cmailCycles;
        cmailV1_tmp = cmailV1;
        cmailT1_tmp = cmailT1;
        cmailV2_tmp = cmailV2;
        cmailT2_tmp = cmailT2;
      }
      if (key == RST) {
        cmailCycleNow = 0;
        cmailSecRemain = 0;
        setpointVolts = 0.0;
        cmailMode = OffMode;
      }
      if (key == RUN_PAUSE) {
        if (cmailMode==OffMode) {
          cmailMode = StartMode;
        } else if (cmailPause) {
          cmailPause = false;
        } else {
          cmailPause = true;
        }
      }
    }
    else if (uiScreen==SetupScreen) {
      const KeyName SEL = F1Key;
      const KeyName OK = F3Key;
      const KeyName CANCL = F4Key;
      if (key == SEL) { 
        if (setupScreenSelector == T2Sel) {
          setupScreenSelector = CyclesSel;
        } else {
          setupScreenSelector = (SetupScreenSelector)(setupScreenSelector + 1);
        }
        // initialize pot channel to existing value
        switch (setupScreenSelector) {
          case CyclesSel:
            analogProcessed[PotChan] = cmailCycles_tmp/9999.0;
          break;
          case V1Sel:
            analogProcessed[PotChan] = cmailV1_tmp/250.0;
          break;
          case T1Sel:
            analogProcessed[PotChan] = cmailT1_tmp/9999.0;
          break;
          case V2Sel:
            analogProcessed[PotChan] = cmailV2_tmp/250.0;
          break;
          case T2Sel:
            analogProcessed[PotChan] = cmailT2_tmp/9999.0;
          break;
        }
      }
      if (key == OK) {
        cmailCycles = cmailCycles_tmp;
        cmailV1 = cmailV1_tmp;
        cmailT1 = cmailT1_tmp;
        cmailV2 = cmailV2_tmp;
        cmailT2 = cmailT2_tmp;
        uiScreen = MainScreen;
      }
      if (key == CANCL) {
        uiScreen = MainScreen;        
      }
    }
  }
  lastKey = key;
}


void updateCMAIL() {
  static unsigned long lastSecondTick;
  if (millis()-lastSecondTick > 1000) {
    lastSecondTick += 1000;
    if (!cmailPause && cmailMode!=OffMode) {
      --cmailSecRemain;
    }
  }
  switch(cmailMode) {
    case StartMode:
      lastSecondTick = millis();
      cmailSecRemain = cmailT1;
      cmailCycleNow = (cmailCycles==0) ? 0 : 1;
      cmailMode = NSMode;
      cmailPause = false;
      break;
    case NSMode:
      setpointVolts = cmailPause ? 0.0 : cmailV1;
      if (cmailSecRemain==0 && cmailCycles>0) {
        cmailMode = WaitEWMode;
        cmailSecRemain = 1;
      }
      break;
    case WaitEWMode:
      setpointVolts = 0.0;
      if (cmailSecRemain==0) {
        cmailMode = EWMode;
        cmailSecRemain = cmailT2;
      }
      break;
    case EWMode:
      setpointVolts = cmailPause ? 0.0 : cmailV2;
      if (cmailSecRemain==0) {
        if (++cmailCycleNow <= cmailCycles) {
          cmailMode = WaitNSMode;
          cmailSecRemain = 1;
        } else {
          cmailMode = OffMode;
          cmailCycleNow = 0;
          cmailSecRemain = 0;
          setpointVolts = 0.0;
        }
      }
      break;
    case WaitNSMode:
      setpointVolts = 0.0;
      if (cmailSecRemain==0) {
        cmailMode = NSMode;
        cmailSecRemain = cmailT1;
      }
      break;
    case OffMode:
      break;
  }
}

void handleSerialInput() { }

void discretize(float input, int ch) {
  static int lastVal[numDiscretes];
  static int debounceCount[numDiscretes];
  const int debounceLimit[numDiscretes] = { 3, 3 };
  const int numKeys = 14;
  const float keyVals[numKeys] = 
    { 223., 276., 329., 381., 433., 485., 537., 588., 640., 691., 794., 845., 899., 950. };
  //   M1    M2    M3    M4    M5    M6    M7    M8    M9   M10    F4    F3    F2    F1
  if (ch == 0) {
    int k = -1;
    for (int i=0; i<numKeys; ++i) {
      if (abs(input-keyVals[i])<15.0) {
        k = i;
        break;
      }
    }
    if (k==lastVal[ch]) {
      if (--debounceCount[ch] == 0) {
        discreteValue[ch] = k;
      }
    }
    else {
      lastVal[ch] = k;
      debounceCount[ch] = debounceLimit[ch];
    }
  }
  // if (ch == 1) ...
}

void linearize(float input, int ch) { };

void analogSmooth(AnalogChan ch, int in);
void analogSmooth(AnalogChan ch, int in) {
  analogSmoothed[ch] *= (1.0 - smoothingK[ch]);
  analogSmoothed[ch] += smoothingK[ch]*(float)in;
}

void readAnalogInputs() {
  static unsigned long last_read_time = 0;
  const float PotVirtualTurns = 100.0;
  float deltaPot, d, t;
  if (millis()-last_read_time > analogReadInterval) {
    last_read_time = millis();
    float lastPot = analogSmoothed[PotChan];
    for (int i = 0; i<ANALOG_CHANS; ++i) {
      AnalogChan inChan = (AnalogChan)i;
      int rawAnalog = analogRead(analogPin[inChan]);
      analogSmooth(inChan, rawAnalog);
      int proc = processVal[inChan];
      switch (proc) {
        case Discrete0:
        case Discrete1:
          discretize(analogSmoothed[inChan], proc-Discrete0);
          break;
        case Linearize0:

          linearize(analogSmoothed[inChan], proc-Linearize0);
          break;
        case Linearize1: // amps sense
          analogProcessed[inChan] = (analogSmoothed[inChan]-52.)/180.;
          if (analogProcessed[inChan]<0.0) { analogProcessed[inChan] = 0.0; }
        case PotProc:
          deltaPot = (analogSmoothed[PotChan]-lastPot)/1024.;
          d = abs(deltaPot);
          if (d<0.001) { deltaPot = 0.0; }  // noise reduction
          if (d>0.02) { deltaPot *= (d/0.02)*(d/0.02)*(d/0.02); } // 3rd power ballistics
          t = analogProcessed[PotChan] + deltaPot/PotVirtualTurns;
          if (t<0.0) { t = 0.0; }
          if (t>1.0) { t = 1.0; }
          analogProcessed[PotChan] = t;
          break;
        case NoProc:
        default:
          analogProcessed[inChan] = analogSmoothed[inChan];
      }
    }

  }
}

