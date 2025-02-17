#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <LedControl.h>
#include <U8g2lib.h>
#include <HX711.h>
#include <PID_v1.h>
#include <Bounce2.h>
#include <EEPROM.h>
#include <math.h> // Added to use fabs()
#include <esp_task_wdt.h>
#include <CRC32.h>

#ifdef ESP32
  #define READ_PROGMEM(x) (x)
#else
  #define READ_PROGMEM(x) ((const char*)pgm_read_word(&(x)))
#endif

// Add missing PROGMEM string definitions
const char MSG_CALIBRATION[] PROGMEM = "CALIBRATION";
const char MSG_PID_SETTINGS[] PROGMEM = "PID_SETTINGS";
const char MSG_BRIGHTNESS[] PROGMEM = "BRIGHTNESS";
const char MSG_BTN_CYCLE_ENTER[] PROGMEM = "Btn1: Cycle, Btn3: Enter";
const char MSG_INSUFFICIENT_FORCE[] PROGMEM = "Insufficient force!";

// Add I2C pins definition
#define I2C_SDA 21  // Default I2C SDA pin for ESP32
#define I2C_SCL 22  // Default I2C SCL pin for ESP32

// ---------------- Updated Pin Definitions for NodeMCU ESP32S DevKit 1 (38-pin) ----------------
#define MUX_S0        32  // Changed from 13
#define MUX_S1        33  // Changed from 12
#define MUX_S2        25  // Changed from 14
#define MUX_S3        26  // Changed from 27

#define HEATER_PWM_PIN 16  // GPIO16 - PWM capable
#define COOLER_PWM_PIN 17  // GPIO17 - PWM capable

#define HX711_DT      13  // Changed from 33 
#define HX711_SCK     14  // Changed from 32

#define LED_DIN       23  // MOSI (GPIO23)
#define LED_CLK       18  // SCK  (GPIO18)
#define LED_CS        5   // SS   (GPIO5)

#define BUTTON1_PIN   27  // Changed from 15
#define BUTTON2_PIN   12  // Changed from 16
#define BUTTON3_PIN   4   // Changed from 18
#define BUTTON4_PIN   15  // Changed from 19

 #define BUZZER_PIN    2   // GPIO2 - PWM capable

/****************** OLED Display Settings ******************/
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1   // Not used with I2C
#define OLED_ADDR     0x3C

// Add after PWM constants
#define BUZZER_CHANNEL 2  // Using channel 2 for buzzer
#define BUZZER_FREQ 2000  // Base frequency 2000Hz
#define BUZZER_RES 8      // 8-bit resolution

/****************** Global Objects ******************/
Adafruit_ADS1115 ads;             // ADS1115 ADC (thermistors via multiplexer)
// Updated to use 2 MAX7219 modules
LedControl lc = LedControl(LED_DIN, LED_CLK, LED_CS, 2);  // Two MAX7219 devices
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE); 
HX711 scale;                      // HX711 for load cell

// Keep PID variables as double since PID library requires it
double heaterInput, heaterOutput, heaterSetpoint;
double coolerInput, coolerOutput, coolerSetpoint;
PID pidHeater(&heaterInput, &heaterOutput, &heaterSetpoint, 2.0, 5.0, 1.0, P_ON_M, DIRECT);
PID pidCooler(&coolerInput, &coolerOutput, &coolerSetpoint, 2.0, 5.0, 1.0, P_ON_M, DIRECT);

// Keep PID tuning variables as double
double heaterKp = 2.0, heaterKi = 5.0, heaterKd = 1.0;
double coolerKp = 2.0, coolerKi = 5.0, coolerKd = 1.0;

// Bounce objects for buttons
Bounce button1 = Bounce();
Bounce button2 = Bounce();
Bounce button3 = Bounce();
Bounce button4 = Bounce();

/****************** System Modes ******************/
// Update Mode enum to add IDLE and MEASURING modes
enum Mode { IDLE, MEASURING, CALIBRATION, PID_SETTINGS, BRIGHTNESS, DEFAULT_MODE };
Mode currentMode = IDLE;   // Start in IDLE mode

/****************** Other Global Variables ******************/
// Reduce array sizes where possible
const uint8_t thermistorChannels[10] = {15, 14, 13, 12, 11, 10, 9, 8, 7, 6};
uint8_t hotChannel = 0;   // Changed from int to uint8_t
uint8_t coldChannel = 1;  // Changed from int to uint8_t
float calibrationData[10];  // Calibration factor for each thermistor channel

// Force setpoint (example value in “units” corresponding to HX711 calibration)
double forceSetpoint = 10.0;
double currentForce = 0.0;

// Timing constants (in milliseconds)
unsigned long measurementInterval = 1000; // 1 sec between measurements
unsigned long settlingTime = 10;            // 10 ms settling after mux switching

// For OLED brightness (we adjust contrast using SSD1306 commands)
uint8_t oledBrightness = 128;  // initial brightness

// Define a debounce delay (non-blocking) for mode transitions
const unsigned long debounceDelay = 300;

// Add global buffer (after other global variables)
#define DISPLAY_BUFFER_SIZE 32  // Increase from 16 to 32
char displayBuffer[DISPLAY_BUFFER_SIZE];  // Global buffer for all display operations

// Add plotter constants after other constant definitions
const char SERIAL_DELIMITER = ',';
const char SERIAL_TERMINATOR = '\n';
const unsigned long PLOT_INTERVAL = 100; // 100ms for smoother plots

// Add measurement start time
unsigned long measurementStartTime = 0;  // Add this line

// 1. Add a proper state machine structure
struct SystemState {
    Mode currentMode;
    unsigned long lastUpdate;
    bool isInitialized;
    float lastTemperatures[10];
    float meanHot;
    float meanCold;
    unsigned long lastPlot;  // Added for plotting
};

SystemState systemState = { 
    MEASURING,   // currentMode
    0,          // lastUpdate
    false,      // isInitialized
    {0},        // lastTemperatures
    0,          // meanHot
    0,          // meanCold
    0           // lastPlot
};

// 2. Reduce global string storage
const char* const MESSAGES[] PROGMEM = { "IDLE", "MEAS", "CAL", "PID", "BRI" };

// Add configuration structure
struct Configuration {
    double heaterPID[3];  // Kp, Ki, Kd
    double coolerPID[3];
    float tempLimitHigh;
    float tempLimitLow;
    float forceLimit;
    uint32_t checksum;
} config;

// Add safety limits
const float ABSOLUTE_TEMP_MAX = 150.0f;
const float ABSOLUTE_TEMP_MIN = -20.0f;
const float FORCE_MAX_N = 100.0f;
const unsigned long SENSOR_TIMEOUT = 1000; // 1 second timeout
const unsigned long WDT_TIMEOUT = 5000;    // 5 second watchdog

// Add error tracking
struct ErrorFlags {
    bool sensorTimeout: 1;
    bool tempOutOfRange: 1;
    bool forceOutOfRange: 1;
    bool configError: 1;
    bool adcError: 1;
    bool watchdogError: 1;  // Added
    bool memoryLow: 1;
    bool i2cError: 1;
    bool displayError: 1;
    bool timeoutError: 1;
} errors;

// Add timeout constants
const unsigned long BUTTON_TIMEOUT = 30000;  // 30 second timeout for button operations
const unsigned long EEPROM_WRITE_INTERVAL = 3600000; // 1 hour between EEPROM writes

// Add EEPROM write tracking
unsigned long lastEEPROMWrite = 0;

// Add PWM constants
const int PWM_FREQ = 5000;
const int PWM_RES = 8;
const int HEATER_CHANNEL = 0;
const int COOLER_CHANNEL = 1;

// Add timing management structure
struct TimingControl {
    unsigned long lastADCRead;
    unsigned long lastDisplayUpdate;
    unsigned long lastDataLog;
    unsigned long lastSafetyCheck;
    const unsigned long ADC_INTERVAL = 100;
    const unsigned long DISPLAY_INTERVAL = 250;
    const unsigned long LOG_INTERVAL = 1000;
    const unsigned long SAFETY_INTERVAL = 500;
};

// Add safety timeout
const unsigned long SAFETY_TIMEOUT = 30000; // 30 seconds

/****************** Function Prototypes ******************/
void selectMultiplexerChannel(int channel);
float convertADCtoTemperature(int16_t adcValue, float calibFactor);
void updateLEDDisplay(double hotTemp, double coldTemp);
void updateOLEDDisplayMeasurement(float hotTemp, float coldTemp, float force);
double readForce();
float readTemperature(uint8_t channel);  // Added
bool isTemperatureStable(float currentHot, float currentCold, float prevHot, float prevCold); // Added
void updateDisplays(float hotTemp, float coldTemp);  // Added
void printTemperaturesToSerial();  // Add this line

Mode idleMode();
Mode measuringMode();
Mode calibrationMode();
Mode pidSettingsMode();
Mode brightnessMode();

// New buzzer alert functions
void buzzerAlertButton();
void buzzerAlertMeasurementFinished();
void buzzerAlertError();
void stopBuzzer();  // Add this line

// Add these functions after existing prototypes
void saveConfiguration();
bool loadConfiguration();
uint32_t calculateChecksum(const Configuration& cfg);
void checkSafetyLimits(float temp, float force);
void initWatchdog();
void updateWatchdog();
void handleErrors();

// Add function prototype before other functions
void waitForButton(Bounce& button, unsigned long timeout);

// Add function prototypes right after other prototypes and before waitForButton()
void checkMemory();
bool checkI2CDevice(uint8_t address);
void readTemperatures();

// Add single implementation of waitForButton after the prototypes and before the first usage
void waitForButton(Bounce& button, unsigned long timeout) {
    unsigned long start = millis();
    while (!button.fell()) {
        button.update();
        if (millis() - start > timeout) {
            errors.sensorTimeout = true;
            return;
        }
        esp_task_wdt_reset();  // Keep watchdog happy
    }
}

// Add helper functions for calibration routines:
void calibrateTwoPoint(int channelIndex) {
    // Reduce buffer size
    sprintf(displayBuffer, "S%d: 0C meas", channelIndex);
    u8g2.clearBuffer();
    u8g2.drawStr(0,10, displayBuffer);
    u8g2.drawStr(0,25, (const char*)F("Btn3 to record"));
    u8g2.sendBuffer();
    waitForButton(button3, BUTTON_TIMEOUT);
    if (errors.sensorTimeout) return;
    selectMultiplexerChannel(thermistorChannels[channelIndex]);
    unsigned long startTime = millis();
    while (millis() - startTime < 20) { }
    int16_t adc0 = ads.readADC_SingleEnded(0);
  
    sprintf(displayBuffer, "S%d: 100C meas", channelIndex);
    u8g2.clearBuffer();
    u8g2.drawStr(0,10, displayBuffer);
    u8g2.drawStr(0,25, (const char*)F("Btn3 to record"));
    u8g2.sendBuffer();
    waitForButton(button3, BUTTON_TIMEOUT);
    if (errors.sensorTimeout) return;
    selectMultiplexerChannel(thermistorChannels[channelIndex]);
    startTime = millis();
    while (millis() - startTime < 20) { }
    int16_t adc100 = ads.readADC_SingleEnded(0);
  
    if (adc100 != adc0)
        calibrationData[channelIndex] = 100.0f / (adc100 - adc0);
    else
        calibrationData[channelIndex] = 1.0f;
  
    sprintf(displayBuffer, "S%d: 2-Point OK", channelIndex);
    u8g2.clearBuffer();
    u8g2.drawStr(0,10, displayBuffer);
    u8g2.sendBuffer();
    startTime = millis();
    while (millis() - startTime < 1000) { }
}

void calibrateOnePoint(int channelIndex) {
    // Reduce buffer size
    sprintf(displayBuffer, "S%d: 0C meas", channelIndex);
    u8g2.clearBuffer();
    u8g2.drawStr(0,10, displayBuffer);
    u8g2.drawStr(0,25, (const char*)F("Btn3 to record"));
    u8g2.sendBuffer();
    waitForButton(button3, BUTTON_TIMEOUT);
    if (errors.sensorTimeout) return;
    selectMultiplexerChannel(thermistorChannels[channelIndex]);
    unsigned long startTime = millis();
    while (millis() - startTime < 20) { }
    
    // Removed unused ADC reading variable.
    // int16_t reference = ads.readADC_SingleEnded(0);
    
    calibrationData[channelIndex] = 1.0f;
    
    sprintf(displayBuffer, "S%d: 1-Point OK", channelIndex);
    u8g2.clearBuffer();
    u8g2.drawStr(0,10, displayBuffer);
    u8g2.sendBuffer();
    startTime = millis();
    while (millis() - startTime < 1000) { }
}

/****************** SETUP ******************/
void setup() {
  Serial.begin(115200);
  delay(100); // Add small delay after Serial
  
  // Initialize I2C first
  Wire.begin(I2C_SDA, I2C_SCL);  // Initialize I2C with correct pins
  Wire.setClock(100000); // Set I2C clock to 100kHz
  delay(100); // Add delay after I2C init
  
  // Initialize OLED with error checking
  Serial.println("Initializing OLED display...");
  if (!u8g2.begin()) {
    Serial.println("OLED initialization failed!");
    errors.displayError = true;
  } else {
    Serial.println("OLED initialization successful");
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x12_tr);
    u8g2.setDrawColor(1);
    u8g2.drawStr(0, 10, "Display OK");
    u8g2.sendBuffer();
    delay(1000);
  }

  // Initialize EEPROM
  EEPROM.begin(512);  // Add this line

  // Debugging statement
  Serial.println("EEPROM initialized");

  // Initialize multiplexer control pins
  pinMode(MUX_S0, OUTPUT);
  pinMode(MUX_S1, OUTPUT);
  pinMode(MUX_S2, OUTPUT);
  pinMode(MUX_S3, OUTPUT);

  // Debugging statement
  Serial.println("Multiplexer control pins initialized");
  
  // Initialize PWM channels for heater and cooler
  ledcSetup(HEATER_CHANNEL, PWM_FREQ, PWM_RES);  // Channel 0, 5kHz, 8-bit resolution
  ledcSetup(COOLER_CHANNEL, PWM_FREQ, PWM_RES);  // Channel 1, 5kHz, 8-bit resolution
  ledcAttachPin(HEATER_PWM_PIN, HEATER_CHANNEL);  // Attach heater to channel 0
  ledcAttachPin(COOLER_PWM_PIN, COOLER_CHANNEL);  // Attach cooler to channel 1

  // Debugging statement
  Serial.println("PWM channels for heater and cooler initialized");
  
  // Initialize HX711 (load cell)
  scale.begin(HX711_DT, HX711_SCK);
  scale.set_scale(2280.f);   // Adjust this calibration factor as needed
  scale.tare();              // Reset scale to 0

  // Debugging statement
  Serial.println("HX711 (load cell) initialized");
  
  // Initialize LED display (MAX7219)
  lc = LedControl(LED_DIN, LED_CLK, LED_CS, 2); // Two MAX7219 devices
  lc.shutdown(0, false);
  lc.setIntensity(0, 8);
  lc.clearDisplay(0);
  lc.shutdown(1, false);
  lc.setIntensity(1, 8);
  lc.clearDisplay(1);

  // Debugging statement
  Serial.println("LED display (MAX7219) initialized");
  
  // Initialize OLED display
  u8g2.begin();
  u8g2.setFont(u8g2_font_6x12_tr);  // Add this line - use a basic font
  u8g2.setDrawColor(1);  // Add this line

  // Debugging statement
  Serial.println("OLED display initialized");
  
  // Initialize ADS1115
  if (!ads.begin()) {
    Serial.println(F("Error: ADS1115 initialization failed!"));
    while(1) { delay(10); } // halt execution
  }
  ads.setGain(GAIN_ONE);        // ±4.096V range
  ads.setDataRate(RATE_ADS1115_128SPS); // Changed from RATE_128_SPS to correct constant

  // Debugging statement
  Serial.println("ADS1115 initialized");
  
  // Initialize Bounce buttons
  pinMode(BUTTON1_PIN, INPUT_PULLUP);
  pinMode(BUTTON2_PIN, INPUT_PULLUP);
  pinMode(BUTTON3_PIN, INPUT_PULLUP);
  pinMode(BUTTON4_PIN, INPUT_PULLUP);
  button1.attach(BUTTON1_PIN);
  button1.interval(25);
  button2.attach(BUTTON2_PIN);
  button2.interval(25);
  button3.attach(BUTTON3_PIN);
  button3.interval(25);
  button4.attach(BUTTON4_PIN);
  button4.interval(25);

  // Debugging statement
  Serial.println("Bounce buttons initialized");
  
  // Initialize the buzzer pin
  pinMode(BUZZER_PIN, OUTPUT);
  ledcSetup(BUZZER_CHANNEL, BUZZER_FREQ, BUZZER_RES);  // Channel 2, 2000 Hz, 8-bit resolution
  ledcAttachPin(BUZZER_PIN, BUZZER_CHANNEL);  // Attach buzzer to channel 2
  ledcWrite(BUZZER_CHANNEL, 0);  // Ensure buzzer is off at start

  // Debugging statement
  Serial.println("Buzzer initialized");
  
  // Set initial PID setpoints (example temperatures in °C)
  heaterSetpoint = 50.0;
  coolerSetpoint = 20.0;
  pidHeater.SetMode(AUTOMATIC);
  pidCooler.SetMode(AUTOMATIC);

  // Debugging statement
  Serial.println("PID controllers initialized");
  
  // Load stored calibration data (if any); here we use a default factor of 1.0.
  for (int i = 0; i < 10; i++) {
    calibrationData[i] = 1.0f;
  }
  
  // Load stored calibrationData from EEPROM
  EEPROM.get(0, calibrationData);
  // Validate calibration factors to ensure they are positive and not NaN
  for (int i = 0; i < 10; i++) {
      if (calibrationData[i] <= 0.0f || isnan(calibrationData[i])) {
          calibrationData[i] = 1.0f;
      }
  }
  
  // Optionally, add validation of data here; for now assume data is valid

  // Debugging statement
  Serial.println("Calibration data loaded from EEPROM");

  // Start in Measuring mode (changed from MEASUREMENT)
  currentMode = MEASURING;

  // Initialize watchdog
  initWatchdog();
  if (errors.watchdogError) {
      Serial.println(F("Warning: Watchdog initialization failed"));
  }

  // Debugging statement
  Serial.println("Watchdog initialized");
  
  // Load configuration
  if (!loadConfiguration()) {
      // Use defaults if config load fails
      heaterKp = 2.0; heaterKi = 5.0; heaterKd = 1.0;
      coolerKp = 2.0; coolerKi = 5.0; coolerKd = 1.0;
  }

  // Debugging statement
  Serial.println("Configuration loaded");
  
  // Initialize error flags
  memset(&errors, 0, sizeof(errors));

  // Add plotter headers
  Serial.println(F("Time,HotTemp,ColdTemp,Force,HeaterPWM,CoolerPWM"));

  // Add I2C device checking
  if (!checkI2CDevice(OLED_ADDR)) {
      errors.i2cError = true;
      Serial.println(F("OLED display not found!"));
  }

  stopBuzzer();  // Ensure buzzer is off after initialization

  // Debugging statement
  Serial.println("Setup complete");
}

/****************** LOOP ******************/
void loop() {
  static TimingControl timing;
  unsigned long currentMillis = millis();

  // Critical tasks
  updateWatchdog();
  handleErrors();
  checkSafetyLimits(systemState.meanHot, (float)readForce());  // Update this line
  checkMemory();

  // Non-critical tasks with different intervals
  if (currentMillis - timing.lastADCRead >= timing.ADC_INTERVAL) {
      readTemperatures();  // Use the new function
      timing.lastADCRead = currentMillis;
  }

  if (currentMillis - timing.lastDisplayUpdate >= timing.DISPLAY_INTERVAL) {
      updateDisplays(systemState.meanHot, systemState.meanCold);
      timing.lastDisplayUpdate = currentMillis;
  }

  // Print temperatures to serial monitor
  printTemperaturesToSerial();  // Add this line

  // Mode handling with safety timeout
  static unsigned long measurementStartTime = 0;
  
  switch(currentMode) {
      case MEASURING:
          if (currentMillis - measurementStartTime > SAFETY_TIMEOUT) {
              errors.timeoutError = true;
              currentMode = IDLE;
          } else {
              currentMode = measuringMode();
          }
          break;
      case IDLE:
          currentMode = idleMode();
          break;
      case CALIBRATION:
          currentMode = calibrationMode();
          break;
      case PID_SETTINGS:
          currentMode = pidSettingsMode();
          break;
      case BRIGHTNESS:
          currentMode = brightnessMode();
          break;
      default:
          currentMode = IDLE;
          break;
  }
}

/****************** Helper Functions ******************/

// Sets the multiplexer channel (0-15) by writing to S0-S3 pins.
void selectMultiplexerChannel(int channel) {
  if (channel < 0 || channel > 15) {
    Serial.println(F("Error: Invalid multiplexer channel!"));
    return;
  }
  digitalWrite(MUX_S0, (channel & 0x01) ? HIGH : LOW);
  digitalWrite(MUX_S1, (channel & 0x02) ? HIGH : LOW);
  digitalWrite(MUX_S2, (channel & 0x04) ? HIGH : LOW);
  digitalWrite(MUX_S3, (channel & 0x08) ? HIGH : LOW);
}

// Updated temperature conversion function specifically for NTC 3950 100k thermistor
float convertADCtoTemperature(int16_t adcValue, float calibFactor) {
    // ADS1115 voltage conversion
    float voltage = (adcValue * 4.096f) / 32767.0f;
    
    Serial.print("ADC="); Serial.print(adcValue);
    Serial.print(" V="); Serial.print(voltage, 3);
    
    // Constants for 100kΩ NTC thermistor with 100kΩ series resistor
    const float R_FIXED = 100000.0f;  // 100kΩ series resistor
    const float V_SUPPLY = 3.3f;      // Supply voltage
    const float BETA = 3950.0f;       // Beta coefficient
    const float T0 = 298.15f;         // 25°C in Kelvin
    const float R0 = 100000.0f;       // 100kΩ at 25°C
    
    if (voltage < 0.1f || voltage >= V_SUPPLY) {
        Serial.println(" V out of range");
        return -273.15f;
    }
    
    // Calculate thermistor resistance using voltage divider (voltage measured across R_FIXED)
    float R_thermistor = R_FIXED * ((V_SUPPLY - voltage) / voltage);
    // Ensure a default calibration factor if invalid
    float effectiveCalib = (calibFactor <= 0.0f) ? 1.0f : calibFactor;
    R_thermistor *= effectiveCalib;
    
    Serial.print(" R="); Serial.print(R_thermistor);
    
    if (R_thermistor <= 0.0f) {
        Serial.println(" R invalid");
        return -273.15f;
    }
    
    // Calculate temperature using Beta parameter equation
    float steinhart = log(R_thermistor / R0);
    steinhart = steinhart / BETA;
    steinhart += 1.0f / T0;
    float temperature = 1.0f / steinhart - 273.15f;
    
    Serial.print(" T="); Serial.print(temperature, 1);
    Serial.println("C");
    
    if (temperature < -55.0f || temperature > 125.0f) {
        return -273.15f;
    }
    
    return temperature;
}

// Update the 7-seg LED display via MAX7219.
// Updated to clear and update module 0 for hot side temperature and module 1 for cold side temperature.
void updateLEDDisplay(double hotTemp, double coldTemp) {
  int hot = (int)hotTemp;
  int cold = (int)coldTemp;
  
  // Clear both modules
  lc.clearDisplay(0);
  lc.clearDisplay(1);
  
  // Display hot temperature on module 0 (e.g., digits 0-3)
  for (int i = 0; i < 4; i++) {
    int digit = hot % 10;
    lc.setDigit(0, i, digit, false);
    hot /= 10;
  }
  
  // Display cold temperature on module 1 (e.g., digits 0-3)
  for (int i = 0; i < 4; i++) {
    int digit = cold % 10;
    lc.setDigit(1, i, digit, false);
    cold /= 10;
  }
}

// In updateOLEDDisplayMeasurement, use smaller buffer and PROGMEM strings
void updateOLEDDisplayMeasurement(float hotTemp, float coldTemp, float force) {
    // Check if display is working
    if (errors.displayError) {
        Serial.println("OLED display error - skipping update");
        return;
    }

    // Debugging statements
    Serial.println("Updating OLED display with measurements");
    
    char tempStr[32];
    u8g2.clearBuffer();
    
    // Add error checking for each draw operation
    snprintf(tempStr, sizeof(tempStr), "Hot: %.1f C", (double)hotTemp);
    if (u8g2.drawStr(0, 12, tempStr) == 0) {
        Serial.println("Failed to draw hot temp");
        errors.displayError = true;
        return;
    }
    
    snprintf(tempStr, sizeof(tempStr), "Cold: %.1f C", (double)coldTemp);
    if (u8g2.drawStr(0, 28, tempStr) == 0) {
        Serial.println("Failed to draw cold temp");
        errors.displayError = true;
        return;
    }
    
    snprintf(tempStr, sizeof(tempStr), "Force: %.1f N", (double)force);
    if (u8g2.drawStr(0, 44, tempStr) == 0) {
        Serial.println("Failed to draw force");
        errors.displayError = true;
        return;
    }
    
    u8g2.sendBuffer();
}

// Read the force value from the HX711 load cell.
double readForce() {
    if (scale.is_ready()) {
        long reading = scale.get_units(5);  // average over 5 readings
        return (double)reading;
    }
    Serial.println(F("Error: HX711 not ready!"));
    return 0.0;
}

// New buzzer alert functions
void buzzerAlertButton() {
    static unsigned long buzzerStart = 0;
    static bool buzzerActive = false;
    
    if (!buzzerActive) {
        ledcWriteTone(BUZZER_CHANNEL, 1000);  // 1kHz tone
        buzzerStart = millis();
        buzzerActive = true;
    } else if (millis() - buzzerStart >= 100) {
        ledcWrite(BUZZER_CHANNEL, 0);  // Stop tone
        buzzerActive = false;
    }
}

void buzzerAlertError() {
    ledcWriteTone(BUZZER_CHANNEL, 500);  // 500Hz tone
    delay(500);
    ledcWrite(BUZZER_CHANNEL, 0);  // Stop tone
}

void buzzerAlertMeasurementFinished() {
    ledcWriteTone(BUZZER_CHANNEL, 1500);  // 1.5kHz tone
    delay(100);
    ledcWrite(BUZZER_CHANNEL, 0);
    delay(100);
    ledcWriteTone(BUZZER_CHANNEL, 1500);
    delay(100);
    ledcWrite(BUZZER_CHANNEL, 0);
}

void stopBuzzer() {
    ledcWrite(BUZZER_CHANNEL, 0);  // Ensure buzzer is off
}

// Add error handling helper
void handleErrors() {
    if (errors.sensorTimeout) {
        buzzerAlertError();
        u8g2.clearBuffer();
        u8g2.drawStr(0, 20, "Sensor Timeout!");
        u8g2.sendBuffer();
        delay(1000);
    }
    // ... handle other errors ...
}

// Add configuration storage functions
void saveConfiguration() {
    unsigned long currentTime = millis();
    if (currentTime - lastEEPROMWrite < EEPROM_WRITE_INTERVAL) {
        return;  // Too soon for another write
    }
    config.heaterPID[0] = heaterKp;
    config.heaterPID[1] = heaterKi;
    config.heaterPID[2] = heaterKd;
    config.coolerPID[0] = coolerKp;
    config.coolerPID[1] = coolerKi;
    config.coolerPID[2] = coolerKd;
    config.checksum = calculateChecksum(config);
    EEPROM.put(sizeof(calibrationData), config);
    EEPROM.commit();
    lastEEPROMWrite = currentTime;
}

bool loadConfiguration() {
    EEPROM.get(sizeof(calibrationData), config);
    if (config.checksum != calculateChecksum(config)) {
        errors.configError = true;
        return false;
    }
    heaterKp = config.heaterPID[0];
    heaterKi = config.heaterPID[1];
    heaterKd = config.heaterPID[2];
    coolerKp = config.coolerPID[0];
    coolerKi = config.coolerPID[1];
    coolerKd = config.coolerPID[2];
    return true;
}

// Add implementation for calculateChecksum
uint32_t calculateChecksum(const Configuration& cfg) {
    // Simple CRC32 implementation
    CRC32 crc;
    // Calculate CRC of everything except the checksum itself
    crc.update((uint8_t*)&cfg, sizeof(Configuration) - sizeof(uint32_t));
    return crc.finalize();
}

// Add safety check function
void checkSafetyLimits(float temp, float force) {
    if (temp > ABSOLUTE_TEMP_MAX || temp < ABSOLUTE_TEMP_MIN) {
        errors.tempOutOfRange = true;
        buzzerAlertError();
    }
    if (force > FORCE_MAX_N) {
        errors.forceOutOfRange = true;
        buzzerAlertError();
    }
}

// Add implementation for updateWatchdog
void updateWatchdog() {
    esp_task_wdt_reset();  // Reset watchdog timer
}

// Add implementation for initWatchdog after other watchdog-related functions
void initWatchdog() {
    esp_err_t err = esp_task_wdt_init(WDT_TIMEOUT / 1000, true);
    if (err != ESP_OK) {
        errors.watchdogError = true;
        return;
    }
    err = esp_task_wdt_add(NULL);
    if (err != ESP_OK) {
        errors.watchdogError = true;
    }
}

// New helper functions
void checkMemory() {
    if (ESP.getFreeHeap() < 10000) {  // 10KB threshold
        errors.memoryLow = true;
        Serial.printf("Low memory: %d bytes\n", ESP.getFreeHeap());
    }
}

bool checkI2CDevice(uint8_t address) {
    Wire.beginTransmission(address);
    return Wire.endTransmission() == 0;
}

bool verifyConfiguration() {
    if (heaterKp <= 0 || heaterKi < 0 || heaterKd < 0 ||
        coolerKp <= 0 || coolerKi < 0 || coolerKd < 0) {
        return false;
    }
    return true;
}

/****************** Mode Functions ******************/

// New function: Idle mode. Wait for user to press button2 (as an example) to start measurement.
Mode idleMode() {
    static unsigned long lastPrint = 0;
    const unsigned long PRINT_INTERVAL = 1000; // Print every second
    unsigned long currentMillis = millis();
    
    u8g2.clearBuffer();
    u8g2.drawStr(0, 20, "IDLE MODE");
    u8g2.drawStr(0, 40, "Press Btn2 to start");
    u8g2.sendBuffer();
    
    // Print all thermistor temperatures periodically
    if (currentMillis - lastPrint >= PRINT_INTERVAL) {
        Serial.println("\n--- Thermistor Temperatures ---");
        for (uint8_t i = 0; i < 10; i++) {
            float temp = readTemperature(i);
            Serial.print("Thermistor ");
            Serial.print(i + 1);
            Serial.print(": ");
            Serial.print(temp, 2);
            Serial.println(" C");
        }
        Serial.println("---------------------------");
        lastPrint = currentMillis;
    }

    button2.update();
    if (button2.fell()) {
        buzzerAlertButton();
        measurementStartTime = millis();
        return MEASURING;
    }
    return IDLE;
}

// 6. Optimized measurement mode (modified)
Mode measuringMode() {
    static unsigned long lastUpdate = 0;
    static unsigned long lastPlot = 0;
    static const unsigned long UPDATE_INTERVAL = 1000;
    static float prevHot = 0, prevCold = 0;
    static bool isFirstMeasurement = true;
    
    unsigned long currentMillis = millis();
    
    // Regular measurement update
    if (currentMillis - lastUpdate >= UPDATE_INTERVAL) {
        // Read temperatures
        float hotSum = 0, coldSum = 0;
        for (uint8_t i = 0; i < 5; i++) {
            hotSum += readTemperature(i);
            coldSum += readTemperature(i + 5);
        }
        
        float currentHot = hotSum / 5.0f;
        float currentCold = coldSum / 5.0f;
        
        systemState.meanHot = currentHot;    // Store for plotting
        systemState.meanCold = currentCold;   // Store for plotting
        
        // Update displays using the new helper function
        updateDisplays(currentHot, currentCold);
        
        // Check stability
        if (!isFirstMeasurement && isTemperatureStable(currentHot, currentCold, prevHot, prevCold)) {
            buzzerAlertMeasurementFinished();
            return IDLE;
        }
        
        prevHot = currentHot;
        prevCold = currentCold;
        isFirstMeasurement = false;
        lastUpdate = millis();

        if (currentMode == MEASURING) {
            // Log data every second
            static unsigned long lastLog = 0;
            if (millis() - lastLog >= 1000) {
                Serial.printf("Time:%lu,Hot:%.2f,Cold:%.2f,Force:%.2f\n",
                    millis(), currentHot, currentCold, readForce());
                lastLog = millis();
            }
        }
    }
    
    // Plotter update at higher frequency
    if (currentMillis - systemState.lastPlot >= PLOT_INTERVAL) {
        // Format: Time,HotTemp,ColdTemp,Force,HeaterPWM,CoolerPWM
        Serial.print(currentMillis);
        Serial.print(SERIAL_DELIMITER);
        Serial.print(systemState.meanHot, 2);
        Serial.print(SERIAL_DELIMITER);
        Serial.print(systemState.meanCold, 2);
        Serial.print(SERIAL_DELIMITER);
        Serial.print(readForce(), 2);
        Serial.print(SERIAL_DELIMITER);
        Serial.print(heaterOutput);
        Serial.print(SERIAL_DELIMITER);
        Serial.print(coolerOutput);
        Serial.print(SERIAL_TERMINATOR);
        
        systemState.lastPlot = currentMillis;
    }
    
    return MEASURING;
}

// ---------------- Calibration Mode ----------------
// Updated Calibration Mode to process all thermistor channels
Mode calibrationMode() {
  unsigned long lastDebounceTime = millis();
  // Use smaller data type for configuration options
  uint8_t option = 0;  // For menu selections
  const char* options[] = { (const char*)F("2-Point"), (const char*)F("1-Point") };
  
  u8g2.clearBuffer();
  u8g2.drawStr(0,10, options[option]);
  u8g2.drawStr(0,25, (const char*)F("Btn1: Cycle, Btn3: Enter"));
  u8g2.sendBuffer();
  
  bool selectDone = false;
  while (!selectDone) {
    button1.update(); button3.update();
    if (button1.fell() && (millis() - lastDebounceTime >= debounceDelay)) {
      lastDebounceTime = millis();
      option = (option + 1) % 2;
      u8g2.clearBuffer();
      u8g2.drawStr(0,10, options[option]);
      u8g2.drawStr(0,25, (const char*)F("Btn1: Cycle, Btn3: Enter"));
      u8g2.sendBuffer();
    }
    if (button3.fell() && (millis() - lastDebounceTime >= debounceDelay)) {
      lastDebounceTime = millis();
      selectDone = true;
    }
  }
  
  // Loop through all channels using the helper functions:
  for (int channelIndex = 0; channelIndex < 10; channelIndex++) {
    sprintf(displayBuffer, "Sensor %d Calib", channelIndex);
    u8g2.clearBuffer();
    u8g2.drawStr(0,10, displayBuffer);
    u8g2.drawStr(0,25, (const char*)F("Press Btn3 to start"));
    u8g2.sendBuffer();
    waitForButton(button3, BUTTON_TIMEOUT);
    if (errors.sensorTimeout) return MEASURING;
    lastDebounceTime = millis();
  
    if (option == 0)
        calibrateTwoPoint(channelIndex);
    else
        calibrateOnePoint(channelIndex);
  }
  
  // Save calibrationData to EEPROM after calibration
  EEPROM.put(0, calibrationData);
  EEPROM.commit();  // Add this line
  
  return MEASURING; // Changed from MEASUREMENT
}

// Use PROGMEM for PID parameter names
const char STR_HEATER_KP[] PROGMEM = "Heater Kp";
const char STR_HEATER_KI[] PROGMEM = "Heater Ki";
const char STR_HEATER_KD[] PROGMEM = "Heater Kd";
const char STR_COOLER_KP[] PROGMEM = "Cooler Kp";
const char STR_COOLER_KI[] PROGMEM = "Cooler Ki";
const char STR_COOLER_KD[] PROGMEM = "Cooler Kd";

// ---------------- PID Settings Mode ----------------
Mode pidSettingsMode() {
  // Define PID parameters for selection.
  struct PIDParam { const char* name; double* value; };  // Changed from float* to double*
  const PIDParam params[] PROGMEM = {
    {STR_HEATER_KP, &heaterKp},
    {STR_HEATER_KI, &heaterKi},
    {STR_HEATER_KD, &heaterKd},
    {STR_COOLER_KP, &coolerKp},
    {STR_COOLER_KI, &coolerKi},
    {STR_COOLER_KD, &coolerKd}
  };
  const int numParams = sizeof(params) / sizeof(params[0]);
  int selected = 0;
  
  bool exitPID = false;
  while (!exitPID) {
    u8g2.clearBuffer();
    sprintf(displayBuffer, "%s: %.1f", READ_PROGMEM(params[selected].name), (double)*(params[selected].value));
    u8g2.drawStr(0,10, displayBuffer);
    u8g2.drawStr(0,25, (const char*)F("Btn1: Cycle"));
    u8g2.drawStr(0,40, (const char*)F("Btn2:+ Btn4:-"));
    u8g2.drawStr(0,55, (const char*)F("Btn3: Exit"));
    u8g2.sendBuffer();
    
    button1.update(); button2.update(); button3.update(); button4.update();
    if (button1.fell()) {
      selected = (selected + 1) % numParams;
      delay(300);
    }
    if (button2.fell()) {
      *(params[selected].value) += 0.1;  // Changed from 0.1f to 0.1
      // Update PID tunings for both controllers
      pidHeater.SetTunings(heaterKp, heaterKi, heaterKd);
      pidCooler.SetTunings(coolerKp, coolerKi, coolerKd);
      delay(200);
    }
    if (button4.fell()) {
      *(params[selected].value) -= 0.1;  // Changed from 0.1f to 0.1
      pidHeater.SetTunings(heaterKp, heaterKi, heaterKd);
      pidCooler.SetTunings(coolerKp, coolerKi, coolerKd);
      delay(200);
    }
    if (button3.fell())
      exitPID = true;
  }
  return MEASURING; // Changed from MEASUREMENT
}

// ---------------- Brightness Settings Mode ----------------
Mode brightnessMode() {
  bool exitBright = false;
  while (!exitBright) {
    u8g2.clearBuffer();
    sprintf(displayBuffer, (const char*)F("Brightness: %d"), oledBrightness);
    u8g2.drawStr(0,10, displayBuffer);
    u8g2.drawStr(0,25, (const char*)F("Btn2: + Btn4: -"));
    u8g2.drawStr(0,40, (const char*)F("Btn3: Exit"));
    u8g2.sendBuffer();
    
    button2.update(); button3.update(); button4.update();
    if (button2.fell()) {
      oledBrightness = min(oledBrightness + 10, 255);
      delay(300);
    }
    if (button4.fell()) {
      oledBrightness = max(oledBrightness - 10, 0);
      delay(300);
    }
    if (button3.fell())
      exitBright = true;
  }
  return MEASURING; // Changed from MEASUREMENT
}

// Updated temperature reading function with improved error handling
float readTemperature(uint8_t channel) {
    static unsigned long lastRead[10] = {0};
    const unsigned long MIN_READ_INTERVAL = 100; // 100ms between readings
    unsigned long currentTime = millis();
    
    if (currentTime - lastRead[channel] < MIN_READ_INTERVAL) {
        return systemState.lastTemperatures[channel];
    }
    
    // Set multiplexer channel and wait for settling
    selectMultiplexerChannel(thermistorChannels[channel]);
    delayMicroseconds(50); // Short delay for mux to settle
    
    // Take multiple samples and average
    const int NUM_SAMPLES = 10;
    int32_t adcSum = 0;
    int validReadings = 0;
    
    // Configure ADS1115 settings
    ads.setGain(GAIN_ONE);        // ±4.096V range
    ads.setDataRate(RATE_ADS1115_128SPS); // Changed from RATE_128_SPS to correct constant
    
    // Take multiple readings
    for (int i = 0; i < NUM_SAMPLES; i++) {
        int16_t adc = ads.readADC_SingleEnded(0);
        if (adc > 0 && adc < 32767) { // Valid reading check
            adcSum += adc;
            validReadings++;
        }
        delayMicroseconds(100); // Short delay between samples
    }
    
    // Check if we got enough valid readings
    if (validReadings < (NUM_SAMPLES / 2)) {
        Serial.printf("Channel %d: Not enough valid readings\n", channel);
        return systemState.lastTemperatures[channel]; // Return last valid reading
    }
    
    // Calculate average ADC value
    int16_t adcAverage = adcSum / validReadings;
    
    // Convert to temperature
    float temp = convertADCtoTemperature(adcAverage, calibrationData[channel]);
    
    // Verify reading is reasonable
    if (temp > -55.0f && temp < 125.0f) {
        systemState.lastTemperatures[channel] = temp;
        lastRead[channel] = currentTime;
    } else {
        Serial.printf("Channel %d: Invalid temperature reading: %.2f°C\n", channel, temp);
        return systemState.lastTemperatures[channel];
    }
    
    return temp;
}

// 5. Improved measurement stability check
bool isTemperatureStable(float currentHot, float currentCold, float prevHot, float prevCold) {
    const float STABILITY_THRESHOLD = 0.1f;
    return (fabs(currentHot - prevHot) <= STABILITY_THRESHOLD &&
            fabs(currentCold - prevCold) <= STABILITY_THRESHOLD);
}

// Add new helper function updateDisplays
void updateDisplays(float hotTemp, float coldTemp) {  
    updateLEDDisplay(hotTemp, coldTemp);
    // Use readForce() for OLED display update:
    updateOLEDDisplayMeasurement(hotTemp, coldTemp, (float)readForce());
}

// Add a function to verify temperature readings
bool verifyTemperatureReading(float temp, uint8_t channel) {
    const float MAX_TEMP_CHANGE = 5.0f; // Maximum reasonable temperature change per second
    static float lastValidTemp[10] = {25.0f}; // Initialize with room temperature
    static unsigned long lastCheckTime[10] = {0};
    
    unsigned long currentTime = millis();
    float timeDelta = (currentTime - lastCheckTime[channel]) / 1000.0f; // Convert to seconds
    
    if (timeDelta > 0) {
        float tempChange = fabs(temp - lastValidTemp[channel]);
        float maxAllowedChange = MAX_TEMP_CHANGE * timeDelta;
        
        if (tempChange > maxAllowedChange) {
            Serial.printf("Warning: Suspicious temperature change on channel %d: %.1f°C in %.1fs\n",
                        channel, tempChange, timeDelta);
            return false;
        }
        
        lastValidTemp[channel] = temp;
        lastCheckTime[channel] = currentTime;
    }
    
    return true;
}

// Update readTemperatures function to handle errors better
void readTemperatures() {
    float hotSum = 0.0f, coldSum = 0.0f;
    int validHot = 0, validCold = 0;
    
    // Read hot side sensors (0-4)
    for (uint8_t i = 0; i < 5; i++) {
        float temp = readTemperature(i);
        if (temp > -55.0f && temp < 125.0f) {
            hotSum += temp;
            validHot++;
        }
    }
    
    // Read cold side sensors (5-9)
    for (uint8_t i = 5; i < 10; i++) {
        float temp = readTemperature(i);
        if (temp > -55.0f && temp < 125.0f) {
            coldSum += temp;
            validCold++;
        }
    }
    
    // Update system state if we have valid readings
    if (validHot > 0) {
        systemState.meanHot = hotSum / validHot;
    }
    if (validCold > 0) {
        systemState.meanCold = coldSum / validCold;
    }
    
    // Debug output
    Serial.printf("Valid sensors - Hot: %d, Cold: %d\n", validHot, validCold);
    Serial.printf("Mean temps - Hot: %.2f°C, Cold: %.2f°C\n", 
                 systemState.meanHot, systemState.meanCold);
}

// Add this function to print temperatures to the serial monitor
void printTemperaturesToSerial() {
    Serial.print("Hot Temp: ");
    Serial.print(systemState.meanHot, 2);
    Serial.print(" C, Cold Temp: ");
    Serial.print(systemState.meanCold, 2);
    Serial.println(" C");
}
