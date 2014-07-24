/*
 * An interface for the Tracer Solar Charge Controller
 * A Tracer MT-5 Display Clone
 * -->  Tracer Comm Code originally from https://github.com/xxv/tracer
 * -->  by Steve Pomeroy (XXV)
 * By Rob Westbrook
 * 2014
 */

// use software serial communication with tracer
#include <SoftwareSerial.h>

// include LCD library
#include <ShiftLCD.h>         // 3 wire shift register LCD library

// include libraries for keypad
#include <Keypad_MC17.h>      // keypad library to use MCP23017 chip
#include <Wire.h>             // for i2c comm with MCP23017 chip
#include <Keypad.h>           // keypad library

// include time library
#include <Time.h>

// include eeprom library
// used to store daily kilowatt hours
#include <EEPROM.h>

// create softeare serial port - digital pins 2 and 3
// Pin 3 = receive; Pin 2 = transmit
SoftwareSerial myserial(3, 2); // RX, TX

// setup LCD pins
// pins used to shift register for LCD
ShiftLCD lcd(10,12,11);

// Tracer communication syncronization head
uint8_t start[] = { 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xEB, 0x90, 0xEB, 0x90, 0xEB, 0x90 };

// Tracer controller ID
uint8_t id = 0x16;

// command string for real time monitoring
//                cmd    len  crchi crclo  end
uint8_t cmd[] = { 0xA0, 0x00, 0xB1, 0xA7, 0x7F };

// tracer data variables
int read = 0;                 // holds number of bytes read
float battery = 0.0;          // 9 battery voltage
float pv =  0.0;              // 11 solar panel voltage !bytes 13-14 are reserved!
float load_current =  0.0;    // 15 load current
float over_discharge =  0.0;  // 17 voltage setting for low discharge
float battery_max =  0.0;     // 19 voltage setting for max battery voltage
uint8_t loadOnOff;            // 21 load on/off
uint8_t loadOverload;         // 22 overload yes/no
uint8_t loadShort;            // 23 load short yes/no
uint8_t soc;                  // 24 state of charge
uint8_t batteryOverload;      // 25 battery overload
uint8_t batteryOverDischarge;  // 26 over discharge yes/no
uint8_t full;                 // 27 battery full indicator
uint8_t charging;             // 28 battery charging indicator
int8_t battery_temp;          // 29 battery temperature
float charge_current = 0.0;   // 30 solar panel current

// buffer to hold received data from Tracer
uint8_t buff[128];

// custom icon displayed when data is being read from Tracer
byte rxIcon[8] = {
	0b11110,
	0b11100,
	0b11100,
	0b10010,
	0b00001,
	0b00000,
	0b00000,
	0b00000
};

// watts variables
float totalWatts = 0.0;     // running total of watts for the day
float averageWatts = 0.0;   // average watts
float wattSeconds = 0.0;    // average watts times elapsed time
float wattHours = 0.0;      // wattSeconds divided by 3600
float pastWH = 0.0;         // holds EEPROM reading
float totalWH = 0.0;        // writes to EEPROM
time_t wattTime = 0;        // beginning time at start of day
time_t wattDay = 0;         // day number
time_t wattElapsedTime = 0; // time past since new day began
int wattSample = 0;         // counts program loops

// amp hour variables
float totalAmps = 0.0;      // running total of amps for the day
float averageAmps = 0.0;    // average amps
float ampSeconds = 0.0;     // average amps times elapsed time
float ampHours = 0.0;       // ampSeconds divided by 3600
float pastAH = 0.0;         // holds EEPROM reading
float totalAH = 0.0;        // writes to EEPROM
time_t ampTime = 0;         // beginning time at start of day
time_t ampDay = 0;          // day number
time_t ampElapsedTime = 0;  // time past since new day began
int ampSample = 0;          // counts program loops

// timing variables
long interval = 5000;       // interval between tracer polls
long lastTime = 0;          // last time of poll

// time variables used throughout program
// !!-- SET DATE HERE --!!
// hr,min,sec,day,month,year
int setHour = 16;            // set time to this hour
int setMinute = 06;          // set time to this minute
int setSecond = 0;           // set time to this second
int setDay = 23;             // set time to this day
int setMonth = 7;            // set time to this month
int setYear = 2014;          // set time to this year
// !!-- SET DATE HERE --!!

time_t systemTime;          // overall time 
time_t systemSecond;        // the second of system time
time_t systemMinute;        // the minute of system time
time_t systemHour;          // the hour of system time
time_t systemDay;           // the day of system time
time_t systemWeekday;       // the day of week of system time
time_t systemMonth;         // the month of system time
time_t systemYear;          // the year of system time
time_t currentDay;          // tracks current day to compare for new day

int firstRun = 1;           // used for all start up functions

// variables used for eeprom
int kwMemAddr = 0;        // address where daily kilowatt total is stored
int ahMemAddr = 10;       // address where daily amp hour total is stored

// keypad  variables
char keyBuff[10];        // buffer for keypad input - 7 digits plus \0
char keyHold[10];        // array to transfer buffer contents into before clearing
char lastKey = '*';      // track the last key pressed
int keyCnt = 0;          // counter
int x = 0;               // used in loops

#define I2CADDR 0x20     // define address for MCP23017 chip

const byte ROWS = 4;     // keypad has four rows
const byte COLS = 4;     // keypad has four columns
char keys[ROWS][COLS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','.','D'}       // changed '#' to '.' for decimal point
};

byte rowPins[ROWS] = {0,1,2,3}; //connect to the row pinouts of the keypad
byte colPins[COLS] = {4,5,6,7}; //connect to the column pinouts of the keypad

// create keypad object
Keypad_MC17 keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS, I2CADDR );

/*********************************
**                              **
** SETUP FUNCTION               **
**                              **
*********************************/
void setup() {
  Serial.begin(9600);          // for debugging
  myserial.begin(9600);        // start up software serial
  lcd.createChar(0,rxIcon);    // create custom LCD character
  lcd.begin(20, 4);            // start up LCD
  keypad.begin();              // start up keypad
  setTime(setHour,setMinute,setSecond,setDay,setMonth,setYear); // hr,min,sec,day,month,year
}

/*********************************
**                              **
** LOOP FUNCTION                **
**                              **
*********************************/
void loop() {
  
  // get current time and see if it's time to poll tracer
  unsigned long currentTime = millis();
  
  char key = keypad.getKey(); // see if there's any keypad entry this loop
  
  if(key != NO_KEY) {         // NO_KEY says no key pressed this loop
    lastKey = key;
    switch (key) {
      case 'A':
        doScreenA(currentTime);
        break;
      case 'B':
        doScreenB(currentTime);
        break;
      case 'C':
        doScreenC(currentTime);
        break;
      case 'D':
        doScreenD(currentTime);
        break;
      default:
        break;
    }
  }
  
  // if the interval time has passed do tracer poll
  if(currentTime - lastTime > interval) {
    lastTime = currentTime;        // save current time for comparison next time
  
    //use bottom LCD line for debug
    lcd.setCursor(0,3);
    lcd.print("                   ");
    lcd.setCursor(0,3);
    lcd.print("Step:");
  
    pollLCDIcon(1);                 // turn on polling icon
    
    if(doTime()) {                 // Step A
      lcd.setCursor(5,3);
      lcd.print("A");
    } else {
      lcd.setCursor(5,3);          // or error
      lcd.print("?");
    }
    if(checkForNewDay()) {         // Step B
      lcd.setCursor(6,3);
      lcd.print("B");
    } else {
      lcd.setCursor(6,3);          // or error
      lcd.print("?");
    }  
    if(sendPollSync()) {           // Step C
      lcd.setCursor(7,3);          // send start data and id
      lcd.print("C");
    } else {
      lcd.setCursor(7,3);          // or error
      lcd.print("?");
    }
    if(sendPollCmd()) {            // Step D
      lcd.setCursor(8,3);          // send command
      lcd.print("D");
    } else {
      lcd.setCursor(8,3);          // or error
      lcd.print("?");
    }
    if(read = readPollData()) {    // Step E
      lcd.setCursor(9,3);          // read data from Tracer
      lcd.print("E");
    } else {
      lcd.setCursor(9,3);          // or error
      lcd.print("?");
    }
    if(processPollData()) {        // Step F
      lcd.setCursor(10,3);         // process received data
      lcd.print("F");
    } else {
      lcd.setCursor(10,3);         // or error
      lcd.print("?");
    }
    if(doMainScreen()) {           // Step G
      lcd.setCursor(11,3);         // display data
      lcd.print("G"); 
    } else {
      lcd.setCursor(11,3);         // or error
      lcd.print("?");
    }
    if(doWatts(pv, charge_current)) {  // Step H
      lcd.setCursor(12,3);
      lcd.print("H");
    } else {
      lcd.setCursor(12,3);         // or error
      lcd.print("?");
    }
    if(doAmps(charge_current)) {   // Step I
      lcd.setCursor(13,3);
      lcd.print("I");
    } else {
      lcd.setCursor(13,3);         // or error
      lcd.print("?");
    }
    if(doDisplayTime()) {          // Step J
      lcd.setCursor(14,3);
      lcd.print("J");
    } else {
      lcd.setCursor(14,3);         // or error
      lcd.print("?");
    }
    
    pollLCDIcon(0);              // turn off polling icon
    }
}

/*********************************
**                              **
** FUNCTION TO CONVERT          **
** DATA TO FLOAT                **
**                              **
*********************************/
float to_float(uint8_t* buffer, int offset){
  unsigned short full = buffer[offset+1] << 8 | buff[offset];
  return full / 100.0;
}

/*********************************
**                              **
** FUNCTION TO CALCULATE CRC    **
**                              **
*********************************/
unsigned short crc(unsigned char *CRC_Buff, unsigned char crc_len) {
  unsigned char crc_i, crc_j, r1, r2, r3, r4;
  unsigned short crc_result;

  r1 =*CRC_Buff;
  CRC_Buff++;

  r2=*CRC_Buff;
  CRC_Buff++;

  for (crc_i = 0; crc_i < crc_len -2; crc_i++) {
    r3 =* CRC_Buff;
    CRC_Buff++;

    for (crc_j=0; crc_j < 8; crc_j++) {
      r4 = r1;
      r1 = (r1<<1);

      if ((r2 & 0x80) != 0) {
        r1++;
      }

      r2 = r2<<1;

      if((r3 & 0x80) != 0) {
        r2++;
      }

      r3 = r3<<1;

      if ((r4 & 0x80) != 0) {
        r1 = r1^0x10;
        r2 = r2^0x41;
      }
    }
  }

  crc_result = r1;
  crc_result = crc_result << 8 | r2;

  return(crc_result);
}

/*********************************
**                              **
** FUNCTION THAT CLEARS DATA    **
** ON EACH READ AND DISPLAYS    **
** NEW DATA                     **
**                              **
*********************************/
void displayData(float data, int col, int row, int place){
  lcd.setCursor(col, row);    // set cursor to correct location
  lcd.print("       ");       // print spaces to clear
  lcd.setCursor(col, row);    // set cursor back to correct location
  lcd.print(data, place);     // print new data
}

/*********************************
**                              **
** FUNCTION TO DISPLAY CUSTOM   **
** ICON WHEN TRACER IS POLLED   **
** onOff VARIABLE:              **
**   1 = TURN ICON ON           **
**   0 = TURN ICON OFF          **
**                              **
*********************************/
void pollLCDIcon(int onOff){
  if(onOff == 1) {            // if icon is turned on
    lcd.setCursor(19,3);      // set cursor to correct location
    lcd.write((uint8_t)0);    // write icon to location
  }
  else                        // if icon is turned off
  {
    lcd.setCursor(19,3);      // set cursor to correct location
    lcd.print(" ");           // print space to clear
  }
}

/*********************************
**                              **
** FUNCTION TO INITIALIZE       **
** TRACER COMMUNICATION         **
** SEND THE START STRING        **
** FOLLOWED BY THE ID           **
**                              **
*********************************/
bool sendPollSync() {
 myserial.write(start, sizeof(start));  // write start string to tracer
  // send ID
  myserial.write(id);                   // write ID to tracer
}

/*********************************
**                              **
** FUNCTION TO SEND COMMAND     **
** STRING TO TRACER             **
**                              **
*********************************/
bool sendPollCmd() {
 // send command string
  myserial.write(cmd, sizeof(cmd));     // write command string to tracer
}

/*********************************
**                              **
** FUNCTION TO READ DATA BACK   **
** FROM TRACER                  **
**                              **
*********************************/
int readPollData() {
 int read = 0;
    for (int i = 0; i < 255; i++){      // cycle through all bytes
      if (myserial.available()) {       // if data was received
        buff[read] = myserial.read();   // write data to buffer
        read++;
      }
    }
 return read;                           // return number of bytes read
}

/*********************************
**                              **
** FUNCTION TO PROCESS RECEIVED **
** DATA FROM TRACER             **
** FLOATS ARE 2 BYTES           **
** UNSIGNED INTEGERS ARE 1 BYTE **
**ASSIGN DATA TO THEIR VARIABLES**
**                              **
*********************************/
bool processPollData() {
  battery = to_float(buff, 9);
  pv = to_float(buff, 11);
  load_current = to_float(buff, 15);
  over_discharge = to_float(buff, 17);
  battery_max = to_float(buff, 19);
  loadOnOff = buff[21];// 21 load on/off
  loadOverload = buff[22];// 22 overload yes/no
  loadShort = buff[23];// 23 load short yes/no
  soc = buff[24];// 24 reserved
  batteryOverload = buff[25];// 25 battery overload
  batteryOverDischarge = buff[26];// 26 over discharge yes/no
  full = buff[27];
  charging = buff[28];
  battery_temp = buff[29];
  charge_current = to_float(buff, 30);
}

/*********************************
**                              **
** FUNCTION TO DISPLAY TRACER   **
** DATA ON LCD MAIN SCREEN      **
**                              **
*********************************/
bool doMainScreen(){
  lcd.setCursor(0,0);
  lcd.print("BV:");
  displayData(battery,3,0,2);
  lcd.setCursor(10,0);
  lcd.print("PV:");
  displayData(pv,13,0,2);
  lcd.setCursor(0,1);
  lcd.print("SOC:");
  displayData(soc,4,1,0);
  lcd.setCursor(10,1);
  lcd.print("PA:");
  displayData(charge_current,13,1,2);
  lcd.setCursor(0,2);
  lcd.print("Wt:");
  float watts = (pv * charge_current);
  displayData(watts,3,2,1);
  lcd.setCursor(10,2);
  lcd.print("Tp:");
   
}

/*********************************
**                              **
** FUNCTION TO DISPLAY DATA     **
** WHEN "A" IS PRESSED ON KEYPAD**
**                              **
*********************************/
void doScreenA(unsigned long startTime) {
  unsigned long nowTime = millis();  // get time for calculating length this screen is shown
  do {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Load A:");
    displayData(load_current,7,0,2);
    lcd.setCursor(0,1);
    lcd.print("Hi V:");
    displayData(over_discharge,5,1,2);
    lcd.setCursor(0,2);
    lcd.print("Batt Full V:");
    displayData(battery_max,12,2,2);
    nowTime = nowTime + millis();
  } while (nowTime - startTime < interval);  // display until time is greater than interval
}

/*********************************
**                              **
** FUNCTION TO DISPLAY DATA     **
** WHEN "B" IS PRESSED ON KEYPAD**
**                              **
*********************************/
void doScreenB(unsigned long startTime) {
  unsigned long nowTime = millis();  // get time for calculating length this screen is shown
  do {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Load On:");
    lcd.setCursor(8,0);
    lcd.print(loadOnOff ? "Y" : "N" );
    lcd.setCursor(0,1);
    lcd.print("Overload:");
    lcd.setCursor(9,1);
    lcd.print(loadOverload ? "Y" : "N" );
    lcd.setCursor(0,2);
    lcd.print("Load Short:");
    lcd.setCursor(11,2);
    lcd.print(loadShort ? "Y" : "N" );
    nowTime = nowTime + millis();
  } while (nowTime - startTime < interval);  // display until time is greater than interval
}

/*********************************
**                              **
** FUNCTION TO DISPLAY DATA     **
** WHEN "C" IS PRESSED ON KEYPAD**
**                              **
*********************************/
void doScreenC(unsigned long startTime) {
  unsigned long nowTime = millis();  // get time for calculating length this screen is shown
  do {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Batt Charge:");
    lcd.setCursor(12,0);
    lcd.print(charging ? "Y" : "N" );
    lcd.setCursor(0,1);
    lcd.print("Batt Full:");
    lcd.setCursor(10,1);
    lcd.print(full ? "Y" : "N" );
    lcd.setCursor(0,2);
    lcd.print("Batt Discharged:");
    lcd.setCursor(16,2);
    lcd.print(batteryOverDischarge ? "Y" : "N" );
    lcd.setCursor(0,3);
    lcd.print("Batt Temp:");
    float battery_f = battery_temp + 30;
    displayData(battery_f,10,3,1);
    nowTime = nowTime + millis();
  } while (nowTime - startTime < interval);  // display until time is greater than interval
}

/*********************************
**                              **
** FUNCTION TO DISPLAY DATA     **
** WHEN "D" IS PRESSED ON KEYPAD**
**                              **
*********************************/
void doScreenD(unsigned long startTime) {
  unsigned long nowTime = millis();  // get time for calculating length this screen is shown
  do {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Watt Hours:");
    displayData(wattHours,11,0,1);
    lcd.setCursor(0,1);
    lcd.print("Total WH:");
    float tempWH = wattHours + EEPROM.read(kwMemAddr);
    displayData(tempWH,9,1,0);
    lcd.setCursor(0,2);
    lcd.print("Amp Hours:");
    displayData(ampHours,10,2,1);
    lcd.setCursor(0,3);
    lcd.print("Total AH:");
    float tempAH = ampHours + EEPROM.read(ahMemAddr);
    displayData(tempAH,9,3,0);
    nowTime = nowTime + millis();
  } while (nowTime - startTime < interval);  // display until time is greater than interval  
}

/*********************************
**                              **
** FUNCTION TO DO ALL           **
** WATT CALCULATIONS            **
**                              **
*********************************/
bool doWatts(float panel_voltage, float panel_current) {
  wattSample = wattSample + 1;                   // increase loop count
  wattElapsedTime = now() - wattTime;            // elapsed time is equal to current time minus watt time value
  float watts = panel_voltage * panel_current;   // calculate current watts
  totalWatts = totalWatts + watts;               // total watts so far
  averageWatts = totalWatts/wattSample;          // average the watts
  wattSeconds = averageWatts * wattElapsedTime;  // average X time
  wattHours = wattSeconds/3600;                  // seconds divided by 3600 (number of seconds in an hour)
}

/*********************************
**                              **
** FUNCTION TO DO ALL           **
** AMP CALCULATIONS             **
**                              **
*********************************/
bool doAmps(float amps) {
  ampSample = ampSample + 1;                     // increase loop count
  ampElapsedTime = now() - ampTime;              // elapsed time is equal to current time minus amp time value
  totalAmps = totalAmps + amps;                  // total amps so far
  averageAmps = totalAmps/ampSample;             // average the amps
  ampSeconds = averageAmps * ampElapsedTime;     // average X time
  ampHours = ampSeconds/3600;                    // seconds divided by 3600 (number of seconds in an hour)
}

/*********************************
**                              **
** FUNCTION TO GET CURRENT TIME **
** AND SET ALL TIME VARIABLES   **
**                              **
*********************************/
bool doTime() {
  systemTime = now();
  systemSecond = second(systemTime);
  systemMinute = minute(systemTime);
  systemHour = hour(systemTime);
  systemDay = day(systemTime);
  systemWeekday = weekday(systemTime);
  systemMonth = month(systemTime);
  systemYear = year(systemTime);
}

/*********************************
**                              **
** FUNCTION TO DISPLAY CURRENT  **
** TIME ON MAIN SCREEN          **
**                              **
*********************************/
bool doDisplayTime() {
  addZero(hourFormat12(systemTime), 10, 2); // display hour in 12 hour format
  lcd.setCursor(12,2);
  lcd.print(":");
  addZero(systemMinute, 13, 2);             // display minute
  isAmPm(15, 2);
}

/*********************************
**                              **
** FUNCTION TO ADD A LEADING    **
** ZERO TO NUMBERS THAT ARE     **
** LESS THAN 10                 **
**                              **
*********************************/
void addZero(int digits, int col, int row) {
  if(digits < 10) {                    // if number less than 10
    lcd.setCursor(col, row);
    lcd.print("0");                    // print a leading zero
    lcd.setCursor(col + 1, row);
    lcd.print(digits);                 // then print number
  } else {
    lcd.setCursor(col, row);
    lcd.print(digits);                 // if number 10 or greater
  }                                    // print as is
}

/*********************************
**                              **
** FUNCTION TO DETERMINE IF     **
** IT'S AM OR PM                **
**                              **
*********************************/
void isAmPm(int col, int row) {
  if(isAM(systemTime)) {
    lcd.setCursor(col, row);
    lcd.print("AM");
  } else {
    lcd.setCursor(col, row);
    lcd.print("PM");
  }
}

/*********************************
**                              **
** FUNCTION TO CHECK IF IT'S    **
** NOW A NEW DAY                **
**                              **
*********************************/
bool checkForNewDay() {
  if(currentDay != systemDay) {
    doNewDay();
  }
}

/*********************************
**                              **
** FUNCTION TO DO IF IT IS A    **
** NEW DAY                      **
**                              **
*********************************/
bool doNewDay() {
  Serial.println("In doNewDay!!");
  doNewDayWatts();
  doNewDayAmps();
  currentDay = systemDay;
}

/*********************************
**                              **
** FUNCTION THAT DOES NEW DAY   **
** ROUTINES FOR WATTS           **
**                              **
*********************************/
bool doNewDayWatts() {
  pastWH = EEPROM.read(kwMemAddr);        // read stored WH total
  totalWH = pastWH + wattHours;           // add today's watt hours with stored total
  EEPROM.write(kwMemAddr, totalWH);       // save total watt hours
  wattSample = 0;                         // a new day so start over samples
  wattElapsedTime = 0;                    // a new day so reset elapsed time
  wattTime = now();                       // a new day so get new seconds reading
  wattDay = systemDay;                    // set to new day 
}

/*********************************
**                              **
** FUNCTION THAT DOES NEW DAY   **
** ROUTINES FOR AMPS            **
**                              **
*********************************/
bool doNewDayAmps() {
  pastAH = EEPROM.read(ahMemAddr);        // read stored AH total
  totalAH = pastAH + ampHours;            // add today's watt hours with stored total
  EEPROM.write(ahMemAddr, totalAH);       // save total watt hours
  ampSample = 0;                          // a new day so start over samples
  ampElapsedTime = 0;                     // a new day so reset elapsed time
  ampTime = now();                        // a new day so get new seconds reading
  ampDay = systemDay;                     // set to new day
}