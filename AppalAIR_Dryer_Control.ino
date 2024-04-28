/*
    AppalAIR Dryer PID System

    This system is centered around the PermaPure MD-700 which operates using a purge flow of dried compressed air
    Using an RH sensor following the Dryer, this program uses PID functionality to calculate a voltage to control the 
    Enfield valve, thus controlling purge flow rate and ultimatley the drying efficiency.

    Hardware: 
    PermaPure Monotube Dryer 700 (MD-700)
    Arduino MKR WiFi 1010 (connected is the MKR485 Shield but unnecessary)
    Breadboard 3x OpAmp Circuit
    HD44780 LCD Display with I2C
    Enfield SS Motorized Needle Valve with FKM Seals (ENV-0375-SSF) with Motor Driver
    Werther PC120/4-C Air Compressor + NANPU 1/2" NPT 4 Stage Desiccant Dryer

    Software:
    WiFi is used to gather universal time and load into the on-chip RTC. This is used for accurate and synced interrupts
    Alarms are set for every dt (seconds) for the PID control, and every minute to update the SetPoint (to mimic CPD3)
    While the program awaits interrupts, it reads %RH data into an array, to average into PV before PID calculation

    Programmer: Shawn Beekman <shawnbeekman1@gmail.com>

    Last Updated April 15, 2024
*/

#include <WiFiNINA.h>
#include <WiFiUdp.h>
#include <RTCZero.h>
#include <LCD_I2C.h>

//////////////////////// GLOBALS ///////////////////////////

const int MAX_OUTPUT = 255, MAX_INPUT = 1023;   // MKR Wifi 1010 Board
const double OP_VOLT = 3.3;                     // Operating Voltage: 3.3V
int A2D_bits = 12;
int A2D_res = pow(2, A2D_bits) - 1;

const int INPUT_RH = A3;                  // RH sensor input pin
const int INPUT_T = A5;                   // T sensor input pin
const int OUTPUT_PIN = A0;                // Enfield output pin

char ssid[] = "asu-visitor";              // Network SSID (name)
WiFiUDP Udp;
unsigned int localPort = 2390;            // Local port to listen for UDP packets
const char *ntpServer = "pool.ntp.org";
unsigned long epochTime;
uint8_t hour, minute, second;
const int NTP_PACKET_SIZE = 48;
byte packetBuffer[NTP_PACKET_SIZE];       // Buffer to hold incoming and outgoing packets

double dt = 2;                            // dt used for setting alarms and setting alarm
double lastTime = 0, lastUpdate;
double integral = 0, previous = 0, output = 0;
double kp = 1.0, ki = 0.1, kd = 0.0;
double PV;                                // Present Value (%RH)
double SP = 100;                          // SetPoint (%RH)

int numSamples = 100;                     // # Samples to take in between interrupts (fit as many as you can)
double PV_Array[numSamples];

// This array is used to set the SP every minute. Must be manually synced with CPD3 configuration.
double setpoint[60] = {01, 01, 01, 01, 01, 01, 01, 35, 35, 40, 40, 44, 44, 48, 48, 52, 55, 58, 61, 63,
                       65, 67, 69, 71, 73, 75, 77, 79, 81, 81, 81, 81, 81, 79, 77, 75, 73, 71, 69, 67,
                       65, 63, 61, 58, 55, 52, 52, 48, 48, 44, 44, 40, 40, 35, 35, 20, 20, 01, 01, 01};

double actualminute[60] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20,
                            21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40,
                            41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 0}

double read_RH1, read_RH2, read_T;
double RH1, RH2, voltage, T;

volatile bool tick_RTC = false;
volatile bool tick_PID = false;
RTCZero rtc;

LCD_I2C lcd(0x27, 20, 4); // 0x27 Default I2CtoLCD Address, 20x4 Screen

/////////////////////// MAIN CODE ///////////////////////////

void setup() {
  // Initialize Serial Ports/Wifi
  Serial.begin(9600);
  while (!Serial) {
    delay(250);
  }

  // Change Read Resolution to 12 bits for better precision
  analogReadResolution(A2D_bits);

  connectToWiFi(); Serial.println("Connected to WiFi");
  Udp.begin(localPort); Serial.println("UDP started");

  // Setup RTC
  rtc.begin();
  updateCurrentTime(); updateSP();
  rtc.setTime(hour, minute, second);

  // Minute Alarm
  rtc.setAlarmSeconds(58);            // 58s to account for delay - allows every interrupt to happen at 0s
  rtc.enableAlarm(rtc.MATCH_SS);
  rtc.attachInterrupt(RTCAlarm);

  // PID Alarm (Every dt seconds)
  for (int i = 0; i < 60; i += dt) {
    rtc.setAlarmSeconds(i);
  }
  rtc.enableAlarm(rtc.MATCH_SS);
  rtc.attachInterrupt(PIDAlarm);

  lcd.begin(); lcd.backlight(); lcd.clear();
  updateLCD();

/*
  // Set up interrupt for every minute
  rtc.setAlarmSeconds(58);
  rtc.enableAlarm(rtc.MATCH_SS);
  rtc.attachInterrupt(RTCAlarm);

  // Set up interrupt for every 15 seconds
  rtc.setAlarmSeconds(13, 28, 43, 58);
  rtc.enableAlarm(rtc.MATCH_SS);
  rtc.attachInterrupt(SP_Alarm);
  */
}

void loop() {
  // Minute Alarm
  if (tick_RTC) {
    hour = rtc.getHours(); minute = rtc.getMinutes(); second = rtc.getSeconds(); // Read time from RTC
    updateSP();
    updateLCD();
    tick_RTC = false;
    integral = 0; // Reset integral every minute
  }

  // PID Alarm ~2s
  if (tick_PID) {
    // Average RH readings for PV
    PV = sum(PV_Array) / len(PV_Array);
    updateLCD();
    // Perform PID calculation
    output = pid(PV-SP);
    if (output > MAX_OUTPUT) { // Coerce
      output = MAX_OUTPUT;
    }
    else if (output < 0) {
      output = 0;
    }
    analogWrite(OUTPUT_PIN, output);
    tick_PID = false;
    sample = 0; // Reset samples
    PV_Array = {};
  }

  // Read RH sensor in between PID ticks
  while (sample < numSamples) {
    PV_Array[sample] = analogRead(INPUT_RH) * 100 * 3.3 / A2D_res; // Read voltage (0-1V) convert to %RH
    sample++;
  }
}

  /*
  // Read in RH, convert input to %RH, convert output to volts and plot
  read_RH1 = analogRead(INPUT_RH);      // 0-4095
  read_RH2 = analogRead(A4);
  //read_T = analogRead(INPUT_T);       // 0-4095
  analogWrite(OUTPUT_PIN, output);
  RH1 = 100 * read_RH1 * 3.3 / A2D_res;
  RH2 = 100 * read_RH2 * 3.3 / A2D_res;
  //T = ((read_T * 3.3 / A2D_res) * 120) - 40;
  voltage = (double) output * 10 / MAX_OUTPUT;
  T = 0;
  Serial.print(voltage); Serial.print(" V, RH_before: "); Serial.print(RH2); Serial.print(", RH_after: "); Serial.println(RH1);
  //Serial.print("RH: "); Serial.println(RH);
  delay(1000);
  */
}


/////////////////////////// FUCNTIONS ////////////////////////////////////////

// PID Function - feedback function to couple %RH with Actuator voltage
// Inputs: Error = PV - SP
// Outputs: Enfield Motorized Actuator Control Signal (to be converted to 0-3.3V)
double pid(double error) {
  double proportional = error;
  integral += error * dt;
  double derivative = (error - previous) / dt;
  previous = error;
  return (kp * proportional) + (ki * integral) + (kd * derivative);
}

// Update SetPoint based on the minute of hour
double updateSP() {
  if (minute == 59) {
    SP = setpoint[0];
  }
  else {
    SP = setpoint[minute+1];
  }
}

// RTC Alarm
void RTCAlarm() {
  tick_RTC = true;
}

// PID Alarm
void PIDAlarm() {
  tick_PID = true;
}

// Update LCD - Display current SetPoint (SP), Present Value (PV), and UTC to LCD
// Inputs: SP, PV, hour, minute
// Outputs: HD44780 LCD Display
void updateLCD() {
  lcd.clear();
  lcd.setCursor(3,0);
  lcd.print("AppalAIR Dryer");
  lcd.setCursor(0,1);
  lcd.print("Time (UTC): "); lcd.print(hour); lcd.print(":"); lcd.print(actualminute[minute]);
  lcd.setCursor(0,2);
  lcd.print("  PV (%RH): "); lcd.print(RH1);
  lcd.setCursor(0,3);
  lcd.print("  SP (%RH): "); lcd.print(SP);
}
//////////////////////////////////////////// WiFi CODE /////////////////////////////////////////////////////
// Connect to WiFi - Joins asu-visitor to talk to NTP (Network Time Protocol)
void connectToWiFi() {
  Serial.print("Connecting to WiFi");
  WiFi.begin(ssid);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }

  Serial.println();
}

// Returns NTP sync time to epochTime variable, hour, minute, seconds
void updateCurrentTime() {
  sendNTPRequest();
  delay(1000);

  if (Udp.parsePacket()) {
    Udp.read(packetBuffer, NTP_PACKET_SIZE);
    unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
    unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
    epochTime = highWord << 16 | lowWord;
    epochTime -= 2208988800UL; // Unix epoch time
    hour = (epochTime / 3600) % 24;
    minute = (epochTime % 3600) / 60;
    second = epochTime % 60;
  }
}


// Retrieve time in packetBuffer
void sendNTPRequest() {
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  packetBuffer[0] = 0b11100011; // LI, Version, Mode
  packetBuffer[1] = 0;          // Stratum, or type of clock
  packetBuffer[2] = 6;          // Polling Interval
  packetBuffer[3] = 0xEC;       // Peer Clock Precision
  packetBuffer[12] = 49;
  packetBuffer[13] = 0x4E;
  packetBuffer[14] = 49;
  packetBuffer[15] = 52;

  Udp.beginPacket(ntpServer, 123); // NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////