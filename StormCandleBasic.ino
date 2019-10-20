
// Displays the relative air pressure on a red (high) - yellow - green (low) scale

// Connect Arduino Nano -> LED:
// D5  -> 220 Ohm -> Red lead
// D6  -> 220 Ohm -> Green lead
// GND -> GND lead

// Connect Arduino Nano -> pressure sensor
// A5  -> SCL
// A4  -> SDA
// GND -> GND
// 5V  -> VCC
// or
// 3.3V -> 3.3V

// Connect Arduino Nano -> LDR
// A6 -> LDR -> 5V
// A6 -> 100 kOhm -> GND

// Connect mode switch
// D7 -> GND (for auto brightness)
// or
// D8 -> GND (for full brightness)

#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_BMP085.h>

Adafruit_BMP085 bmp;

//#define DEBUG 1

#define PIN_RED   5 // D5
#define PIN_GREEN 6 // D6
#define PIN_SWITCH_MODE_A 7 // D7
#define PIN_SWITCH_MODE_B 8 // D8
#define PIN_LDR A6
#define MEDIAN_COUNT 7 // Number of measurements per loop
#define STARTUP_RANGE 20 // Distance (in Pascale) between initial measurement and maximum / minimum at startup

int  i;
int  iMedian;
long pa_measurements[MEDIAN_COUNT]; 
long pa_current;
long pa_max;
long pa_min;
long pa_center;
float led_max = 255;

int  led_red   = 0;
int  led_green = 0;

int  ldr_sensor;
bool switch_mode_a;
bool switch_mode_b;

int addr_long_hpa_min = 0;
int addr_long_hpa_max;


void setup() {
  #ifdef DEBUG
    Serial.begin(9600);
  #endif

  pinMode(PIN_SWITCH_MODE_A, INPUT_PULLUP);
  pinMode(PIN_SWITCH_MODE_B, INPUT_PULLUP);

  // Calculate memory addresses
  addr_long_hpa_max = addr_long_hpa_min + sizeof(long);
  
  if (!bmp.begin()) {
    #ifdef DEBUG
      Serial.println("Sensor not found");
    #endif
    
    while (1) {}
  }

  iMedian = MEDIAN_COUNT / 2;
  
  EEPROM.get(addr_long_hpa_min, pa_min);
  EEPROM.get(addr_long_hpa_max, pa_max);

  // Init pressure measurements
  pa_current = bmp.readPressure();

  if(-1 == pa_min) {
    #ifdef DEBUG
      Serial.println("Initializing EEPROM with first min/max values");
    #endif
    
    pa_max = pa_current + STARTUP_RANGE;
    pa_min = pa_current - STARTUP_RANGE;
    pa_center = pa_current;
    
    EEPROM.put(addr_long_hpa_min, pa_min);
    EEPROM.put(addr_long_hpa_max, pa_max);
  } else {
    #ifdef DEBUG
      Serial.println("Loaded min/max values from EEPROM");
    #endif
    
    calculateCenter();
  }
}
  
void loop() {
  processModeSwitch();
  measureAndCalculate();
  setLed();

  #ifdef DEBUG
    //debugLDR();
    //debugLeds();
    debugPressures();
    Serial.println("");    
  #endif
}

void processModeSwitch() {
  switch_mode_a = !digitalRead(PIN_SWITCH_MODE_A);
  switch_mode_b = !digitalRead(PIN_SWITCH_MODE_B);

  // Choose brightness mode ON/OFF/AUTO (Air pressure will still be measured in OFF state)
  if(switch_mode_a) {
    // Brightness AUTO
    ldr_sensor = analogRead(PIN_LDR);
    led_max = ldr_sensor / 4;
  } else if(switch_mode_b) {
    // Brightness ON
    led_max = 255;
  } else {
    // Brightness OFF
    led_max = 0;
  }
}

void measureAndCalculate() {
  // Collect measurements
  for (i = 0; i < MEDIAN_COUNT; i++) {
    pa_measurements[i] = bmp.readPressure();
  }

  // Sort measurements to find the median (center) value(s)
  bubbleSort();
  
  // Average the 3 central values to avoid excess flickering
  pa_current = (pa_measurements[iMedian] + pa_measurements[iMedian - 1] + pa_measurements[iMedian + 1]) / 3;

  // Calculate minimum, maximum and center values
  if(pa_current > pa_max) {
    pa_max = pa_current;
    EEPROM.put(addr_long_hpa_max, pa_max);
  }
  
  if(pa_current < pa_min) {
    pa_min = pa_current;
    EEPROM.put(addr_long_hpa_min, pa_min);
  }
  
  calculateCenter();
}

void calculateCenter() {
  pa_center = pa_min + (pa_max - pa_min) / 2;
}

void setLed() {
  // Set LED colours
  if (pa_current >= pa_center) led_red = led_max; else led_red = led_max * (pa_current - pa_min) / (pa_center - pa_min);
  if (pa_current <= pa_center) led_green = led_max; else led_green = led_max * (pa_max - pa_current) / (pa_max - pa_center);

  analogWrite(PIN_RED, led_red);
  analogWrite(PIN_GREEN, led_green);
}

// Copied and adapted from https://www.tigoe.com/pcomp/code/arduinowiring/42/
void bubbleSort() {
  long out, in, swapper;
  
  for (out=0 ; out < MEDIAN_COUNT; out++) {
    for (in=out; in<(MEDIAN_COUNT-1); in++) {
      if ( pa_measurements[in] > pa_measurements[in+1] ) {
        swapper = pa_measurements[in];
        pa_measurements [in] = pa_measurements[in+1];
        pa_measurements[in+1] = swapper;
      }
    }
  }
}

#ifdef DEBUG
  void debugLDR() {
    Serial.print(0);
    Serial.print("\t");
    Serial.print(ldr_sensor);
    Serial.print("\t");
    Serial.print(1024);
  }
  
  void debugLeds() {
    Serial.print(0);
    Serial.print("\t");
    Serial.print(led_red);
    Serial.print("\t");
    Serial.print(led_green);
    Serial.print("\t");
    Serial.print(255);
  }
  
  void debugPressures() {
    Serial.print(pa_current);
    Serial.print("\t");
    Serial.print(pa_min);
    Serial.print("\t");
    Serial.print(pa_max);
    Serial.print("\t");
    Serial.print(pa_center);
  }
#endif
