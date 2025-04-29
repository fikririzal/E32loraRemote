#include <Arduino.h>

// #define lora Serial2

#define setPoint 0
#define propotional 1
#define integral 2
#define derivative 3

#define setPointMAX 12.0f
#define propotionalMAX 1.00f
#define integralMAX 1.00f
#define derivativeMAX 1.00f

#define throttle 0
#define stearing 1

#define forward 0
#define backward 1

// put function declarations here:
int myFunction(int, int);

#define totalAnalogPin 6
uint8_t analogPin[totalAnalogPin] = {36, 39, 34, 35, 32, 33};
float fPidParm[4] = {0.0f, 0.0f, 0.0f, 0.0f};
uint16_t pidParm[4] = {0, 0, 0, 0};
uint16_t joystickPos[2] = {0, 0};
uint16_t rawJoystickPos[2] = {0, 0};
uint16_t rawPidParm[4] = {0, 0, 0, 0};
boolean throttleDir = 0;
float alpha = .1;
uint16_t stearingLow = 30, stearingHigh = 3887 - 30, throttleLow = 41, throttleHigh = 4065;

uint32_t lastDebug = 0;

volatile bool adc_coversion_done = false;
adc_continuous_data_t *result = NULL;
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
} send_t;

HardwareSerial lora(2);

send_t data2send = {
    .index = 0x55,
    .indetifier = 0xFF,
    ._stearing = 0,
    ._setPoint = 0,
    ._propotional = 0xAFC,
    ._integral = 0xEBF,
    ._derivative = 0xBFC,
    .checkSum = 0xCAF};

uint32_t lastSend = 0;

float fMap(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  lora.begin(115200, SERIAL_8N1, 18, 17);

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
  if (cuurrentTime - lastSend > 30)
  {
    lastSend = cuurrentTime;
    // -----------------------------------------------------
    data2send.checkSum = 0;
    uint16_t checksum = 0;

    checksum += data2send.index;
    checksum += data2send._stearing;
    checksum += data2send._setPoint;
    checksum += data2send._propotional;
    checksum += data2send._integral;
    checksum += data2send._derivative;

    checksum &= 0xFFF;
    data2send.checkSum = checksum;

    // lora.println("Hello, World!!!");
    lora.write((uint8_t *)&data2send, sizeof(data2send));
    // Serial.println("Sended");
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

      joystickPos[throttle] = constrain(map(rawJoystickPos[throttle], stearingLow, stearingHigh, 0, 4095), 0, 4095);
      joystickPos[stearing] = constrain(map(rawJoystickPos[stearing], stearingLow, stearingHigh, 0, 4095), 0, 4095);
      pidParm[setPoint] = constrain(map(rawPidParm[setPoint], 30, 4065, 0, 4095), 0, 4095);
      pidParm[propotional] = constrain(map(rawPidParm[propotional], 30, 4065, 0, 4095), 0, 4095);
      pidParm[integral] = constrain(map(rawPidParm[integral], 30, 4065, 0, 4095), 0, 4095);
      pidParm[derivative] = constrain(map(rawPidParm[derivative], 30, 4065, 0, 4095), 0, 4095);

      fPidParm[setPoint] = fMap(pidParm[setPoint], 0.0f, 4095.0f, 0.0f, setPointMAX);
      fPidParm[propotional] = fMap(pidParm[propotional], 0.0f, 4095.0f, 0.0f, propotionalMAX);
      fPidParm[integral] = fMap(pidParm[integral], 0.0f, 4095.0f, 0.0f, integralMAX);
      fPidParm[derivative] = fMap(pidParm[derivative], 0.0f, 4095.0f, 0.0f, derivativeMAX);

      if (cuurrentTime - lastDebug > 250)
      {
        lastDebug = cuurrentTime;

        // -------------------------------------------------
        Serial.print("Thrt: ");
        Serial.print(joystickPos[0]);
        Serial.print(" | Ster: ");
        Serial.println(joystickPos[1]);

        Serial.print("SP: ");
        Serial.print(fPidParm[setPoint],5);
        Serial.print(" | P: ");
        Serial.print(fPidParm[propotional],5);
        Serial.print(" | I: ");
        Serial.print(fPidParm[integral],5);
        Serial.print(" | D: ");
        Serial.println(fPidParm[derivative],5);
#ifdef JOYSTICK_CALIBRATION
        Serial.print(stearingLow);
        Serial.print(" - ");
        Serial.print(rawJoystickPos[0]);
        Serial.print(" - ");
        Serial.print(stearingHigh);
        Serial.println(" Steering");
        if (rawJoystickPos[0] < stearingLow)
        {
          stearingLow = rawJoystickPos[0];
        }
        if (rawJoystickPos[0] > stearingHigh)
        {
          stearingHigh = rawJoystickPos[0];
        }

        Serial.print(throttleLow);
        Serial.print(" - ");
        Serial.print(rawJoystickPos[1]);
        Serial.print(" - ");
        Serial.print(throttleHigh);
        Serial.println(" Throttel");
        if (rawJoystickPos[1] < throttleLow)
        {
          throttleLow = rawJoystickPos[1];
        }
        if (rawJoystickPos[1] > throttleHigh)
        {
          throttleHigh = rawJoystickPos[1];
        }
#endif
      }
      analogContinuousStart();
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