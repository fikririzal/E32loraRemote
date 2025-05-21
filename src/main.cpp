#include <Arduino.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

// #define lora Serial2

#define setPoint 0
#define propotional 1
#define integral 2
#define derivative 3

#define setPointMAX 4.0f
#define propotionalMAX 2.00f
#define integralMAX 2.00f
#define derivativeMAX 2.00f

#define throttle 0
#define stearing 1

#define forward 0
#define backward 1

// put function declarations here:
int myFunction(int, int);

#define totalAnalogPin 6
uint8_t analogPin[totalAnalogPin] = {36, 39, 34, 35, 32, 33};
float fPidParm[4] = {0.0f, 0.0f, 0.0f, 0.0f};
int16_t SteerMid = 0;
uint16_t scaledSetPoint = 0;
uint16_t pidParm[4] = {0, 0, 0, 0};
uint16_t joystickPos[2] = {0, 0};
uint16_t rawJoystickPos[2] = {0, 0};
uint16_t rawPidParm[4] = {0, 0, 0, 0};
boolean throttleDir = 0;
float alpha = .1;
uint16_t stearingLow = 0+10, stearingHigh = 3835 - 10, throttleLow = 10+10, throttleHigh = 4095-10;

uint32_t lastDebug = 0;

volatile bool adc_coversion_done = false;
adc_continuous_data_t *result = NULL;

uint8_t currModePin = 0;
uint8_t filteredModePin = 0;
uint8_t lastmodePin = 0;
uint8_t sendPIDdata = 0;
uint32_t debounchMode = 0;
uint32_t timeJoystick = 0;
#define modePin 23

#define COLUMS    20 //LCD columns
#define ROWS      4  //LCD rows

LiquidCrystal_I2C lcd(PCF8574_ADDR_A21_A11_A01, 4, 5, 6, 16, 11, 12, 13, 14, POSITIVE);


void ARDUINO_ISR_ATTR adcComplete()
{
  adc_coversion_done = true;
}

typedef struct __attribute__((packed))
{
  uint8_t index;
  uint8_t indetifier;
  uint16_t _stearing : 12;
  uint16_t _setPoint : 12;
  uint16_t _propotional : 12;
  uint16_t _integral : 12;
  uint16_t _derivative : 12; // 68 bit
  uint16_t checkSum : 12;    // 80 bit with chech sum 10byte
} sendPID_t;

typedef struct __attribute__((packed))
{
  uint8_t index;
  uint8_t indetifier;
  uint16_t _stearing : 12;
  uint16_t _setPoint : 12;
  uint8_t dummy:4;
  uint16_t checkSum : 12;    // 80 bit with chech sum 10byte
} sendControl_t;

HardwareSerial lora(2);
                     
sendPID_t dataPID2send = {
    .index = 0x55,
    .indetifier = 0xFF,
    ._stearing = 1000,
    ._setPoint = 1500,
    ._propotional = 2000,
    ._integral = 2500,
    ._derivative = 3000,
    .checkSum = 0
};

sendControl_t dataControl2send = {
  .index = 0x55,
  .indetifier=0x36 ,   
  ._stearing = 2000,
  ._setPoint = 0,
  .dummy = 0,
  .checkSum = 0
};

uint32_t lastSend = 0;
uint32_t intervalSend = 0;

float fMap(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  lora.begin(115200, SERIAL_8N1, 18, 17);

  pinMode(modePin,INPUT_PULLUP);
  currModePin = digitalRead(modePin);

  while (lcd.begin(COLUMS, ROWS, LCD_5x8DOTS) != 1) //colums, rows, characters size
  {
    Serial.println(F("PCF8574 is not connected or lcd pins declaration is wrong. Only pins numbers: 4,5,6,16,11,12,13,14 are legal."));
    delay(5000);   
  }
  lcd.backlight();

  lcd.setCursor(0,0);
	lcd.print("|------------------|");
  lcd.setCursor(0,1);
	lcd.print("|Long Range Remotte|");
	lcd.setCursor(0,2);
	lcd.print("|w/ Telemetry & PID|");
	lcd.setCursor(0,3);
	lcd.print("|------------------|");
  delay(2000);
  lcd.clear();

  analogContinuousSetWidth(12);
  analogContinuousSetAtten(ADC_11db);
  analogContinuous(analogPin, totalAnalogPin, 5, 20000, &adcComplete);
  analogContinuousStart();

  int result = myFunction(2, 3);
}

void loop()
{
  // put your main code here, to run repeatedly:
  uint32_t cuurrentTime = millis();
  if (cuurrentTime - debounchMode > 50)
  {
    debounchMode = cuurrentTime;
    currModePin = digitalRead(modePin);
    if (currModePin != lastmodePin)
    {
      lastmodePin = currModePin;
    } else{
      dataControl2send.dummy = filteredModePin = currModePin;
    }
  }
  
  if (cuurrentTime - lastSend >= 30)
  {
    // intervalSend = cuurrentTime - lastSend;
    // lastSend = cuurrentTime;
    // -----------------------------------------------------
    uint16_t checksum = 0;

    if (!sendPIDdata)
    {
      dataControl2send.checkSum = 0;

      checksum += dataControl2send.index;
      checksum += dataControl2send.indetifier;
      checksum += dataControl2send._stearing;
      checksum += dataControl2send._setPoint;
      checksum += dataControl2send.dummy;
  
      checksum &= 0xFFF;
      dataControl2send.checkSum = checksum;
  
      lora.write((uint8_t *)&dataControl2send, sizeof(dataControl2send)); 
      intervalSend = cuurrentTime - lastSend;
    lastSend = cuurrentTime; 
    }

    if (sendPIDdata)
    {
      sendPIDdata = 0;
      dataPID2send.checkSum = 0;

      checksum += dataPID2send.index;
      checksum += dataPID2send.indetifier;
      checksum += dataPID2send._stearing;
      checksum += dataPID2send._setPoint;
      checksum += dataPID2send._propotional;
      checksum += dataPID2send._integral;
      checksum += dataPID2send._derivative;
  
      checksum &= 0xFFF;
      dataPID2send.checkSum = checksum;
  
      lora.write((uint8_t *)&dataPID2send, sizeof(dataPID2send));
      intervalSend = cuurrentTime - lastSend;
    lastSend = cuurrentTime; 
      Serial.println("sendPIDdata"); 

      lcd.setCursor(0,0);
      lcd.print("|------------------|");
      lcd.setCursor(0,1);
      lcd.print("|PID Data has been |");
      lcd.setCursor(0,2);
      lcd.print("|Send              |");
      lcd.setCursor(0,3);
      lcd.print("|------------------|");
      delay(1000);
      lcd.clear();
    }

  }

  if (cuurrentTime - lastDebug > 250)
  {
    lastDebug = cuurrentTime;

    // -------------------------------------------------
    // Serial.print("Thrt: ");
    // Serial.print(dataPID2send._setPoint);
    // Serial.print(" | Ster: ");
    // Serial.println(dataPID2send._stearing);

    // Serial.print("SP: ");
    // Serial.print(fPidParm[setPoint],5);
    // Serial.print(" | P: ");
    // Serial.print(fPidParm[propotional],5);
    // Serial.print(" | I: ");
    // Serial.print(fPidParm[integral],5);
    // Serial.print(" | D: ");
    // Serial.print(fPidParm[derivative],5);
    // Serial.print(" | Mid: ");
    // Serial.println(SteerMid);

    Serial.print("intervalSend: ");
    Serial.println(intervalSend);

    char line0[21]="    P     I     D   ";
    char line1[21];
    char line2[21]="  SetPoint  Stering ";
    char line3[21];

    //               "  1.123 1.123 1.234 " 
    snprintf (line1,21,"  %01.3f %01.3f %01.3f ",fPidParm[propotional],fPidParm[integral],fPidParm[derivative]);
    
    //            " 1.123m/S   1234   " 
    snprintf (line3,21,"  %01.3fm/S   %04d   ",fMap(dataPID2send._setPoint,0.0f,4095.0f,0.0f,setPointMAX),dataPID2send._stearing);

    lcd.setCursor(0,0);
  	lcd.print(line0);

    lcd.setCursor(0,1);
    // lcd.printf("%01.3f %01.3f %01.3f",fPidParm[propotional],fPidParm[integral],fPidParm[derivative]);
	  lcd.print(line1);

	  lcd.setCursor(0,2);
	  lcd.print(line2);

	  lcd.setCursor(0,3);
	  lcd.print(line3);

    // Serial.print("SP: ");
    // Serial.print(pidParm[setPoint]);
    // Serial.print(" | P: ");
    // Serial.print(pidParm[propotional]);
    // Serial.print(" | I: ");
    // Serial.print(pidParm[integral]);
    // Serial.print(" | D: ");
    // Serial.print(pidParm[derivative]);
    // Serial.print(" | filteredModePin: ");
    // Serial.print(filteredModePin);
    // Serial.print(" | timeJoystick: ");
    // Serial.println(timeJoystick);

// #ifdef JOYSTICK_CALIBRATION
    // Serial.print(stearingLow);
    // Serial.print(" - ");
    // Serial.print(rawJoystickPos[stearing]);
    // Serial.print(" - ");
    // Serial.print(stearingHigh);
    // Serial.println(" Steering");
    // if (rawJoystickPos[stearing] < stearingLow)
    // {
    //   stearingLow = rawJoystickPos[stearing];
    // }
    // if (rawJoystickPos[stearing] > stearingHigh)
    // {
    //   stearingHigh = rawJoystickPos[stearing];
    // }

    // Serial.print(throttleLow);
    // Serial.print(" - ");
    // Serial.print(rawJoystickPos[throttle]);
    // Serial.print(" - ");
    // Serial.print(throttleHigh);
    // Serial.println(" Throttel");
    // if (rawJoystickPos[throttle] < throttleLow)
    // {
    //   throttleLow = rawJoystickPos[throttle];
    // }
    // if (rawJoystickPos[throttle] > throttleHigh)
    // {
    //   throttleHigh = rawJoystickPos[throttle];
    // }
// #endif
  }

  if (adc_coversion_done == true)
  {
    adc_coversion_done = false;
    if (analogContinuousRead(&result, 0))
    {
      analogContinuousStop();
      rawJoystickPos[throttle] = result[0].avg_read_raw;
      rawJoystickPos[stearing] = result[1].avg_read_raw;
      rawPidParm[setPoint] = result[2].avg_read_raw;
      rawPidParm[propotional] = result[3].avg_read_raw;
      rawPidParm[integral] = result[4].avg_read_raw;
      rawPidParm[derivative] = result[5].avg_read_raw;
      SteerMid = constrain(map(analogRead(25), 0, 4095, -490, 490), -490, 490);
      // SteerMid = map(analogRead(25), 0, 4095, -40, 40);
      // SteerMid = analogRead(25);

      pidParm[setPoint] = constrain(map(rawPidParm[setPoint], 30, 4065, 0, 4095), 0, 4095);
      dataPID2send._propotional = pidParm[propotional] = constrain(map(rawPidParm[propotional], 30, 4065, 0, 4095), 0, 4095);
      dataPID2send._integral = pidParm[integral] = constrain(map(rawPidParm[integral], 30, 4065, 0, 4095), 0, 4095);
      dataPID2send._derivative = pidParm[derivative] = constrain(map(rawPidParm[derivative], 30, 4065, 0, 4095), 0, 4095);

      fPidParm[setPoint] = fMap(pidParm[setPoint], 0.0f, 4095.0f, 0.0f, setPointMAX);
      fPidParm[propotional] = fMap(pidParm[propotional], 0.0f, 4095.0f, 0.0f, propotionalMAX);
      fPidParm[integral] = fMap(pidParm[integral], 0.0f, 4095.0f, 0.0f, integralMAX);
      fPidParm[derivative] = fMap(pidParm[derivative], 0.0f, 4095.0f, 0.0f, derivativeMAX);

      scaledSetPoint = joystickPos[throttle] = constrain(map(rawJoystickPos[throttle], throttleLow, throttleHigh, 0, 4095), 0, 4095);
      // dataPID2send._setPoint = joystickPos[throttle] = constrain(map(rawJoystickPos[throttle], throttleLow, throttleHigh, 0, 4095), 0, 4095);
      dataControl2send._setPoint= dataPID2send._setPoint = map(scaledSetPoint, 0, 4095, 0, pidParm[setPoint]);
      dataControl2send._stearing=dataPID2send._stearing = joystickPos[stearing] = constrain(map(rawJoystickPos[stearing], stearingLow, stearingHigh, 500, 4095-500), 500, 4095-500)+SteerMid;
      // data2send._stearing = joystickPos[stearing];

      analogContinuousStart();

      if (filteredModePin == 0&& rawJoystickPos[throttle] < 10 &&rawJoystickPos[stearing] <10)
      {
        if (cuurrentTime - timeJoystick >2000)
        {
          sendPIDdata = 1;
          timeJoystick = cuurrentTime;
        }
        
      } if(rawJoystickPos[throttle] > 10 &&rawJoystickPos[stearing] >10){
        timeJoystick = cuurrentTime;
      }
      
    }
    else
    {
      Serial.println("Error occurred during reading data. Set Core Debug Level to error or lower for more information.");
    }
  }
}

// put function definitions here:
int myFunction(int x, int y)
{
  return x + y;
}