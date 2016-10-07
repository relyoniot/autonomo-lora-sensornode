// TODO
// powersaving: decrese sensor interval, decrese send interval, disable charge led

#include <Sodaq_RN2483.h>
#include <StringLiterals.h>
#include <Switchable_Device.h>
#include <Utils.h>
#include <Sodaq_SHT2x.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Sodaq_wdt.h>
#include <RTCZero.h>

#define debugSerial SerialUSB
#define loraSerial Serial1

#define DEBUG   //If you comment this line, the DPRINT & DPRINTLN lines are defined as blank.
#ifdef DEBUG
  #define DPRINT(...)    debugSerial.print(__VA_ARGS__)     //DPRINT is a macro, debug print
  #define DPRINTLN(...)  debugSerial.println(__VA_ARGS__)   //DPRINTLN is a macro, debug print with new line
#else
  #define DPRINT(...)     //now defines a blank line
  #define DPRINTLN(...)   //now defines a blank line
#endif

//These constants are used for sensor communication
#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10
#define SEALEVELPRESSURE_HPA (1013.25)

//These constants are used for reading the battery voltage
#define ADC_AREF 3.3
#define BATVOLTPIN BAT_VOLT
#define BATVOLT_R1 4.7
#define BATVOLT_R2 10

// Change the DevAddr
const uint8_t devAddr[4] =
{
  0x63, 0xCC, 0xB1, 0xB0
};

// TTN app key
const uint8_t appSKey[16] =
{
  0x9E, 0x17, 0x87, 0x6B, 0x16, 0x09, 0x3F, 0x08, 0x7F, 0x89, 0x83, 0xDB, 0x32, 0xC3, 0xE2, 0xA2
};

// TTN network key
const uint8_t nwkSKey[16] =
{
  0xD1, 0x18, 0xA5, 0x46, 0xA3, 0xC6, 0x03, 0x59, 0x42, 0x55, 0xA4, 0xFF, 0x0A, 0x9E, 0xD5, 0x22
};

//TPH BMP sensor
Adafruit_BME280 bme; // I2C

//RTC
RTCZero rtc;

// global counter
unsigned int count = 0;
unsigned int runminute = 6;
unsigned int runminutetmp = 1;


void setup()
{
  // Startup delay, do NOT remove!
  delay(10000);

  // setup real time clock for sleepmode
  setupRtc();
  
  // Enable WDT
  sodaq_wdt_enable(WDT_PERIOD_8X); //9=4s, 10 = 8s, etc
 
  // initialize digital led pin 13 as an output.
  pinMode(13, OUTPUT);
  
  // init serial ports
  loraSerial.begin(LoRaBee.getDefaultBaudRate());
  debugSerial.begin(57600);
  // Print start message
  debugSerial.println("Start");
  
  // setup lora network
  setupNetwork();
  
  // Connect the sensor
  if (!bme.begin()) {
    debugSerial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }
}

void loop()
{
  if (sodaq_wdt_flag) {
    DPRINTLN("Reset from WDT.");
    sodaq_wdt_flag = false;
    sodaq_wdt_reset();
  }
  DPRINTLN("Measure sensors.");
  // take humidity reading
  float humidity = bme.readHumidity();
  // take temp reading
  float celcius = bme.readTemperature();
  // get batery voltage in mv
  int mv = getRealBatteryVoltage() * 1000.0;
  
  // print result
  DPRINTLN("Humidity: " + String(humidity, DEC) + " %");
  DPRINTLN("Temperature: " + String(celcius, DEC) + " Celcius");
  DPRINTLN("Battery: " + String(mv, DEC)+ " mv");
  DPRINTLN("Interval: " + String(runminute, DEC)+ " minute");
  
  // pack in bytes
  uint16_t value1 = (uint16_t)(celcius * 100); // e.g. 2150
  uint16_t value2 = (uint16_t)(humidity * 100); // e.g. 2150
  uint16_t value3 = (uint16_t)(mv);
  uint16_t value4 = (uint16_t)(runminute);
  
  uint8_t buf[8];
  buf[0] = highByte(value1);
  buf[1] = lowByte(value1);
  buf[2] = highByte(value2);
  buf[3] = lowByte(value2);
  buf[4] = highByte(value3);
  buf[5] = lowByte(value3);
  buf[6] = highByte(value4);
  buf[7] = lowByte(value4);
  
  // Send packet
#ifdef DEBUG
  digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
#endif
    
  // send
  switch (LoRaBee.send(1, buf, sizeof(buf)))
  {
    case NoError:
      receiveData();
      DPRINTLN("Successful transmission");
      break;
    case NoResponse:
      DPRINTLN("There was no response from the device.");
      break;
    case Timeout:
      DPRINTLN("Connection timed-out. Check your serial connection to the device!");
      break;
    case PayloadSizeError:
      DPRINTLN("The size of the payload is greater than allowed. Transmission failed!");
      break;
    case InternalError:
      DPRINTLN("Something is really wrong! Trying a reinit of the network");
      setupNetwork();
      break;
    case Busy:
      DPRINTLN("The device is busy.");
      break;
    case NetworkFatalError:
      DPRINTLN("There is a non-recoverable error with the network connection. Trying a reinit of the network");
      setupNetwork();
      break;
    case NotConnected:
      DPRINTLN("The device is not connected to the network. Trying a reinit of the network");
      setupNetwork();
      break;
    case NoAcknowledgment:
      DPRINTLN("There was no acknowledgment sent back!");
      break;
    default:
      break;
  }
#ifdef DEBUG
  digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
#endif
  // reset wdt
  sodaq_wdt_reset();
  // wait for next 1 minute cycle
  while (count < runminute){
    count++;
    deepSleep();
  }
  count = 0;
}

void deepSleep() {
   
#ifdef DEBUG
  sodaq_wdt_safe_delay(6000);
#else
  // Disable USB
  USB->DEVICE.CTRLA.reg &= ~USB_CTRLA_ENABLE;
  // disable wdt
  sodaq_wdt_disable();
  //Enter sleep mode
  __WFI();
  // ...Sleep
  // Enable WDT
  sodaq_wdt_enable(WDT_PERIOD_8X); //9=4s, 10 = 8s, etc
  // Enable USB
  USB->DEVICE.CTRLA.reg |= USB_CTRLA_ENABLE;
#endif
}

void setupRtc() {
  rtc.begin(1);
  // Set the alarm at the 10 second mark
  rtc.setAlarmSeconds(10);
  // Match only seconds (Periodic alarm every minute)
  rtc.enableAlarm(RTCZero::MATCH_SS);
  // Set sleep mode
  SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
}

void setupNetwork() {
  // initialize digital BEE pin as an output.
  pinMode(BEE_VCC, OUTPUT);
  // Turn the LoRaBee off
  digitalWrite(BEE_VCC, LOW);
  sodaq_wdt_safe_delay(500);  
  // Turn the LoRaBee on
  digitalWrite(BEE_VCC, HIGH);
  // Connect the LoRabee
  LoRaBee.setDiag(debugSerial);
  Serial1.write("mac set dr 0\r\n");
  Serial1.write("radio set pwr 14\r\n");
  Serial1.write("radio set freq 868100000\r\n");
  Serial1.write("radio set sf sf7\r\n");
  
  // Connenct to The Things Network
  if (LoRaBee.initABP(loraSerial, devAddr, appSKey, nwkSKey, true))
    DPRINTLN("Connection to the network was successful.");
  else {
    DPRINTLN("Connection to the network failed!");
  }
}

void receiveData() {
  // we can only receive data just after we send it
  // create recieve buffer
  uint8_t payload[64];
  // get data
  uint16_t len = LoRaBee.receive(payload, 64);
  String HEXPayload = "";
  // if no payload lora will return 131 in payload[0]
  if (payload[0] != 131) {
    for (int i = 0; i < len; i++) {
      HEXPayload += String(payload[i], HEX);
    }
    DPRINTLN("Received payload");
    // get only first number
    runminutetmp = hexchartoint(HEXPayload[1]);
    if (runminutetmp != 0) {
      runminute = runminutetmp;
    }
  } else {
    DPRINTLN("no payload");
  }
}

float getRealBatteryVoltage()
{
  uint16_t batteryVoltage = analogRead(BATVOLTPIN);
  return (ADC_AREF / 1023.0) * (BATVOLT_R1 + BATVOLT_R2) / BATVOLT_R2 * batteryVoltage;
}

unsigned int hexchartoint(char hex) {
    if (hex >= '0' && hex <= '9')
        return hex - '0';

    if (hex >= 'A' && hex <= 'F')
        return hex - 'A';

    if (hex >= 'a' && hex <= 'f')
        return hex - 'a';

    return 0;
}
