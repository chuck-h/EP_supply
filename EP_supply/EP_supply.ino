#include <HID.h>

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

typedef enum                          {AmpCmd, VoltCmd, OVoltCmd, EnableOut, NSOut, EWOut, NUM_OUT_PINS} OutPinType;
const uint8_t outputPin[NUM_OUT_PINS] = {10,      9,       11,        7,        5,     6 }; 

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

enum {OffMode, StartMode, AmpZeroMode, WaitNSMode, WaitEWMode, NSMode, EWMode, TOTAL_MODES} cmailMode;
char modeCode[TOTAL_MODES][3] = {"--", "--", "az", "..", "..", "NS", "EW"};

// Analog input & processing
const unsigned long analogReadInterval = 50; // millisec between analog updates
typedef enum  { NoProc, PotProc, Discrete, SlopeIntercept } ProcessType;
typedef enum              {PotChan=0, KeysChan, StatusChan, VoltsChan, AmpsChan, RemoteChan, ANALOG_CHANS} AnalogChan;
const uint8_t analogPin[] = { A0,      A5,        A4,        A2,         A1,       A3 };
const float   smoothingK[]= { 0.5,     0.5,       0.5,       0.1,        0.1,      0.1};
const int8_t  processVal[]= { PotProc,Discrete,Discrete,SlopeIntercept,SlopeIntercept,SlopeIntercept};
float    analogSmoothed[ANALOG_CHANS];
float    analogProcessed[ANALOG_CHANS];

typedef enum { KeysDiscr, StatusDiscr, NUM_DISCRETES } DiscreteType;
int discreteValue[NUM_DISCRETES];

typedef enum     {VoltsSI, AmpsSI, RemoteSI, NUM_SLOPE_INTERCEPTS } SIType;
struct SIParameters {
  float slope, intercept;
  AnalogChan chan;
};
struct SIParameters siParameters[NUM_SLOPE_INTERCEPTS] = {
  {0.305, 0., VoltsChan}, { 0.0055, -0.29, AmpsChan}, {0.305, 0., RemoteChan} }; 

typedef enum     {AmpsPWMCal, VoltsPWMCal, OVoltsPWMCal, NUM_CALS } CalType;
const int CAL_COEFFS = 4;
struct CalParameters {
  float coeff[CAL_COEFFS];
};
struct CalParameters calParameters[NUM_CALS] = {
  { { 0.0, 0.0, 65000./5., 3000. } }, { { 0.0, 0.0, 65000./300., 0.0 } }, { { 0.0, 0.0, 255./300., 0.0 }, } }; 

struct CmailParameters {
  float v1, v2;
  unsigned int t1, t2, cycles;
};
struct CmailParameters cmailActive, cmailTemp;
const int cmailEEPROMBase = 0;
// CMAIL running values
int cmailCycleNow, cmailSecRemain;
bool cmailPause;

typedef enum {None, CyclesSel, V1Sel, T1Sel, V2Sel, T2Sel} SetupScreenSelector;
SetupScreenSelector setupScreenSelector;

float setpointVolts;

void setup()
{

  for (int p=0; p<NUM_OUT_PINS; ++p) {
    pinMode(outputPin[p], OUTPUT);
  }
  analogWrite(outputPin[OVoltCmd], 250); // set OV to maximum
  setupPWM16();
  analogWrite16(outputPin[VoltCmd], 0);
  analogWrite16(outputPin[AmpCmd], 0);
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

  uint8_t* p = (uint8_t*)&cmailActive;
  for (int i=0; i<sizeof(cmailActive); ++i) {
     *(p+i) = EEPROM.read(i+cmailEEPROMBase);
  }

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

// polynomial calibration curve for PWM outputs
float applyCal(float input, CalType c);
float applyCal(float input, CalType c) {
  CalParameters& cal = calParameters[c];
  float rv = cal.coeff[0];
  for (int i=1; i<CAL_COEFFS; ++i) {
    rv = input*rv + cal.coeff[i]; 
  }
  return rv;
}

void updateOutputs() {
  digitalWrite(outputPin[EnableOut], (cmailMode==OffMode) ? LOW : HIGH);
  digitalWrite(outputPin[NSOut], (cmailMode==NSMode) ? HIGH : LOW);
  digitalWrite(outputPin[EWOut], (cmailMode==EWMode) ? HIGH : LOW);
  float voltPWM = applyCal(setpointVolts, VoltsPWMCal);
  analogWrite16(outputPin[VoltCmd], (uint16_t)voltPWM);
  analogWrite(outputPin[OVoltCmd], 250);
  const float ampsLimit=0.3;
  float ampPWM = applyCal(ampsLimit, AmpsPWMCal);
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
        dtostrf(analogProcessed[VoltsChan], 5, 1, voltString);
        dtostrf(analogProcessed[AmpsChan], 4, 2, ampString);
        sprintf(lcdLine, " %s V %s A [%s]", voltString, ampString, modeCode[cmailMode]);
        break;
      case 1:
        sprintf(lcdLine, " #%4d/%-4d %4d sec", cmailCycleNow, cmailActive.cycles, abs(cmailSecRemain));
        break;
      case 2:
        sprintf(lcdLine, "--------------------");
        break;
      case 3:
        sprintf(lcdLine, "SETUP      RST %s", (cmailMode==OffMode || cmailPause) ? " RUN " : "PAUSE");
    }
  };
  if (uiScreen == SetupScreen) {
    char voltString[6];
    const char parens[2] = {'(', ')'}, spaces[2] = {' ', ' '};
    const char *sel1, *sel2;
    lcd.setCursor (0, line); // go to start of line
    switch (line) {
      case 0:
        sel1 = (setupScreenSelector==CyclesSel) ? parens : spaces;
        sprintf(lcdLine, "  %c%4d%c Cycles [%s]", sel1[0], cmailTemp.cycles, sel1[1], modeCode[cmailMode]);
        break;
     case 1:
        dtostrf(cmailTemp.v1, 5, 1, voltString);
        sel1 = (setupScreenSelector==V1Sel) ? parens : spaces;
        sel2  = (setupScreenSelector==T1Sel) ? parens : spaces;
        sprintf(lcdLine, "NS%c%s%cV %c%4d%csec",sel1[0], voltString, sel1[1], sel2[0], cmailTemp.t1, sel2[1]);
        break;
     case 2:
        dtostrf(cmailTemp.v2, 5, 1, voltString);
        sel1 = (setupScreenSelector==V2Sel) ? parens : spaces;
        sel2 = (setupScreenSelector==T2Sel) ? parens : spaces;
        sprintf(lcdLine, "EW%c%s%cV %c%4d%csec",sel1[0], voltString, sel1[1], sel2[0], cmailTemp.t2, sel2[1]);
        break;
     case 3:
        sprintf(lcdLine, " SEL  ADJ  OK  CANCL");
    }     
  };
  line = (line+1)%4;
  lcd.print(lcdLine);
}
  
void handlePanelKnob() {
  if (setupScreenSelector==CyclesSel) {
    cmailTemp.cycles= int(9999.0*analogProcessed[PotChan]);
  }
  if (setupScreenSelector==V1Sel) {
    cmailTemp.v1 = 250.0*analogProcessed[PotChan];
  }
  if (setupScreenSelector==T1Sel) {
    cmailTemp.t1 = int(9999.0*analogProcessed[PotChan]);
  }
  if (setupScreenSelector==V2Sel) {
    cmailTemp.v2 = 250.0*analogProcessed[PotChan];
  }
  if (setupScreenSelector==T2Sel) {
    cmailTemp.t2 = int(9999.0*analogProcessed[PotChan]);
  }
};


void handleKeyPress() {
  static int lastKey;
  typedef enum {F4Key=10, F3Key, F2Key, F1Key} KeyName;
  int key = discreteValue[KeysDiscr];
  if (lastKey==-1 && key!=-1) {
    // key down
    if (uiScreen==MainScreen) {
      const KeyName SETUP = F1Key;
      const KeyName RST = F3Key;
      const KeyName RUN_PAUSE = F4Key;
      if (key == SETUP) {
        uiScreen = SetupScreen;
        setupScreenSelector = None;
        cmailTemp = cmailActive;
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
            analogProcessed[PotChan] = cmailTemp.cycles/9999.0;
            break;
          case V1Sel:
            analogProcessed[PotChan] = cmailTemp.v1/250.0;
            break;
          case T1Sel:
            analogProcessed[PotChan] = cmailTemp.t1/9999.0;
            break;
          case V2Sel:
            analogProcessed[PotChan] = cmailTemp.v2/250.0;
            break;
          case T2Sel:
            analogProcessed[PotChan] = cmailTemp.t2/9999.0;
            break;
        }
      }
      if (key == OK) {
        cmailActive = cmailTemp;
        // save to EEPROM
        uint8_t* p = (uint8_t*)&cmailActive;
        for (int i=0; i<sizeof(cmailActive); ++i) {
          EEPROM.write(i+cmailEEPROMBase, *(p+i));
        }
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
      cmailSecRemain = 5;
      cmailCycleNow = (cmailActive.cycles==0) ? 0 : 1;
      cmailMode = AmpZeroMode;
      cmailPause = false;
      break;
    case AmpZeroMode:
      setpointVolts = 0.0;
      if (cmailSecRemain == 0) {
        setAmpsZero();
        cmailSecRemain = cmailActive.t1;
        cmailMode = NSMode;
      }
      break;
    case NSMode:
      setpointVolts = cmailPause ? 0.0 : cmailActive.v1;
      if (cmailSecRemain==0 && cmailActive.cycles>0) {
        cmailMode = WaitEWMode;
        cmailSecRemain = 1;
      }
      break;
    case WaitEWMode:
      setpointVolts = 0.0;
      if (cmailSecRemain==0) {
        cmailMode = EWMode;
        cmailSecRemain = cmailActive.t2;
      }
      break;
    case EWMode:
      setpointVolts = cmailPause ? 0.0 : cmailActive.v2;
      if (cmailSecRemain==0) {
        if (++cmailCycleNow <= cmailActive.cycles) {
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
        cmailSecRemain = cmailActive.t1;
      }
      break;
    case OffMode:
      break;
  }
}

void handleSerialInput() { }

void discretize(float input, AnalogChan ch);
void discretize(float input, AnalogChan ch) {
  static int lastVal[NUM_DISCRETES];
  static int debounceCount[NUM_DISCRETES];
  const int debounceLimit[NUM_DISCRETES] = { 3, 3 };
  const int numKeys = 14;
  const float keyVals[numKeys] = 
    { 223., 276., 329., 381., 433., 485., 537., 588., 640., 691., 794., 845., 899., 950. };
  //   M1    M2    M3    M4    M5    M6    M7    M8    M9   M10    F4    F3    F2    F1
  if (ch == KeysChan) {
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
  // if (ch == StatusChan) ...
}

void slopeIntercept(float input, AnalogChan ch);
void slopeIntercept(float input, AnalogChan ch) {
  for (SIParameters* p=siParameters; (p-siParameters)<NUM_SLOPE_INTERCEPTS; ++p) {
    if (p->chan == ch) {
      float val = p->slope*input + p->intercept;
      if (val<0.0) { val = 0.0; }
      analogProcessed[ch] = val;
      return;
    }
  }
};

void setAmpsZero() {
  SIParameters* p=&siParameters[AmpsSI];
  p->intercept = -p->slope*analogSmoothed[AmpsChan];
}

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
        case Discrete:
          discretize(analogSmoothed[inChan], inChan);
          break;
        case SlopeIntercept:
          slopeIntercept(analogSmoothed[inChan], inChan);
          break;
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

