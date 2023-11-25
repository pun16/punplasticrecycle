#include <AccelStepper.h>
#include <MultiStepper.h>
#include <PID_v1.h>
#include "lilParser.h"

// # put in lcd display for information
// # implament heater time out from extruder not running
// # implament interger base command/pramater



//G0 print all setting
//G1 toggle extrution
//G2 set extruder speed
//G3 Set puller speed
//G4 toggle PID puller
//G10 temp heater0
//G11 temp heater1
//G12 temp heater2



char verson = "Verson 0.1 check verson of python code for compatablity";

#define  MAX_PARAMS  4  // The Max number of params that a G-code can have.

// What param types that you can parse out. Looking at G codes it looks like the
// first letter gives the type. I gessed at some. you will need to fill this out.
enum paramTypes {feed, puller, letterI, letterJ, letterK};

// Handy little struct to hold a param value.
// Probably need to modify this at some point.
struct paramValue {
  paramTypes  theType;
  float       theValue;
};

// Our list of G code commands.
enum commands {   noCommand,
                  G0,
                  G1,
                  G2,
                  G3,
                  G4,
                  G10,
                  G11,
                  G12
              };

lilParser   GCodeParser;        // The parser object.



// Define analog pins
const int thermistorPin0 = A0;
const int thermistorPin1 = A1;
const int thermistorPin2 = A2;

const int hallPin = A3;



// Defind smooting diameter varable
#define smooth 100.0
float dia = 1.7;

#define numtemps 6    //Tablecount 

int16_t dia_table[numtemps][2] = {
  //{ADC reading in, diameter out [um]}
  //Init Values for the first start
  //After init Values are read from EEPROM

  { 0     , 3000 },  // safety
  { 619   , 2090 }, //2mm drill bit
  { 702   , 1700 }, //1.7mm
  { 817   , 1400 }, //1.4mm
  { 1000  , 1000 }, // 1mm
  { 1023  , 0000 } //safety
};

// Defind temp smooting varable
const int numReadings  = 10;

int readings0 [numReadings];
int readIndex0  = 0;
long total0  = 0;

int readings1 [numReadings];
int readIndex1  = 0;
long total1  = 0;

int readings2 [numReadings];
int readIndex2  = 0;
long total2  = 0;



// Define heater control pins
const int heaterPin0 = 9;
const int heaterPin1 = 10;
const int heaterPin2 = 11;
const int heaterPin3 = 12;

// Define PID parameters
double Setpoint0 = 0; // Setpoint for heater 0 (A0)
double Setpoint1 = 0; // Setpoint for heater 1 (A1)
double Setpoint2 = 0; // Setpoint for heater 2 (A2)

double Input0, Output0;
double Input1, Output1;
double Input2, Output2;

float P0 = 2;
float I0 = 2;
float D0 = 2;

float P1 = 2;
float I1 = 2;
float D1 = 2;

float P2 = 2;
float I2 = 2;
float D2 = 2;

float mintemp = 10;
float maxtemp = 270;

float R1 = 10000;
float logR2, R2, T;
float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;

float T0, T1, T2;

// Create PID objects
PID myPID0(&Input0, &Output0, &Setpoint0, P0, I0, D0, 0);
PID myPID1(&Input1, &Output1, &Setpoint1, P1, I1, D1, 0);
PID myPID2(&Input2, &Output2, &Setpoint2, P2, I2, D2, 0);

float P3 = 2;
float I3 = 2;
float D3 = 2;

// Create PID objects for filament feedback
double Setpoint3 = 1.75; // Setpoint for diamator sensor (A3)

double Input3, Output3;

PID myPID3(&Input3, &Output3, &Setpoint3, P3, I3, D3, 0);

// Since microstepping is set externally, make sure this matches the selected mode
// If it doesn't, the motor will move at a different RPM than chosen
// 1=full step, 2=half step etc.
#define MICROSTEPS 16

#define SLEEPe 22
#define DIRe 23
#define STEPe 3

#define SLEEPp 24
#define DIRp 25
#define STEPp 4

AccelStepper Extruder(1 , STEPe , DIRe);
AccelStepper Puller(1 , STEPp , DIRp);

float maxPullerSpeed = 1000 / MICROSTEPS;

float extruderSpeed = 50;
float pullerSpeed = 100;

bool doPullerPID = 0;

bool runExtruder = 0;

unsigned long time1;
unsigned long time2;
unsigned long telapsed;

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  Serial.println("starting");

  pinMode(SLEEPe, OUTPUT);
  pinMode(SLEEPp, OUTPUT);

  digitalWrite(SLEEPe , LOW);
  digitalWrite(SLEEPp , LOW);

  Extruder.setMaxSpeed(maxPullerSpeed);
  Puller.setMaxSpeed(maxPullerSpeed);

  // Initialize PID controllers
  myPID0.SetMode(AUTOMATIC);
  myPID1.SetMode(AUTOMATIC);
  myPID2.SetMode(AUTOMATIC);
  myPID3.SetMode(AUTOMATIC);

  // Set heater control pins as OUTPUT
  pinMode(heaterPin0, OUTPUT);
  pinMode(heaterPin1, OUTPUT);
  pinMode(heaterPin2, OUTPUT);

  GCodeParser.addCmd(G0, "G0");
  GCodeParser.addCmd(G1, "G1");
  GCodeParser.addCmd(G2, "G2");
  GCodeParser.addCmd(G3, "G3");
  GCodeParser.addCmd(G4, "G4");
  GCodeParser.addCmd(G10, "G10");
  GCodeParser.addCmd(G11, "G11");
  GCodeParser.addCmd(G12, "G12");
  Serial.println(verson);
}



void loop() {
  time1 = millis();
  char  inChar;
  int   command;
  if (Serial.available() > 0) {
    inChar = Serial.read();                // Read out a charactor.
    Serial.println(inChar);                  // If using development machine, echo the charactor.
    command = GCodeParser.addChar(inChar); // Try parsing what we have.
    switch (command) {                     // Check the results.
      case noCommand : break;             // Nothing to report, move along.
      case G0   : doG0();   break;
      case G1   : doG1();   break;
      case G2   : doG2();   break;
      case G3   : doG3();   break;
      case G4   : doG4();   break;
      case G10   : doG10();   break;
      case G11   : doG11();   break;
      case G12   : doG12();   break;
      default     :
        Serial.println("Invalid command");
        break;
    }
  }


  // Read thermistor values
  int thermistorValue0 = ReadthermistorPin0();
  int thermistorValue1 = ReadthermistorPin1();
  int thermistorValue2 = ReadthermistorPin2();

  // Change thermistor value to tempure
  R2 = R1 * (1023.0 / (float)thermistorValue0 - 1.0);
  logR2 = log(R2);
  T = (1.0 / (c1 + c2 * logR2 + c3 * logR2 * logR2 * logR2));
  T0 = T - 273.15;

  R2 = R1 * (1023.0 / (float)thermistorValue1 - 1.0);
  logR2 = log(R2);
  T = (1.0 / (c1 + c2 * logR2 + c3 * logR2 * logR2 * logR2));
  T1 = T - 273.15;

  R2 = R1 * (1023.0 / (float)thermistorValue2 - 1.0);
  logR2 = log(R2);
  T = (1.0 / (c1 + c2 * logR2 + c3 * logR2 * logR2 * logR2));
  T2 = T - 273.15;

  //    if (maxtemp < T0 || mintemp > T0 || maxtemp < T1 || mintemp > T1 || maxtemp < T2 || mintemp > T2) {
  //      Serial.print("outside operating tempure! Shutting down.");
  //      digitalWrite(heaterPin0 , 0);
  //      digitalWrite(heaterPin1 , 0);
  //      digitalWrite(heaterPin2 , 0);
  //      digitalWrite(SLEEPe , HIGH);
  //      digitalWrite(SLEEPp , HIGH);
  //      Serial.println("Shutdown all part going to suspention.");
  //      while (1) {}
  //    }

  // Control heaters using PID with direction control
  //    Input0 = T0;
  //    myPID0.Compute();
  //    analogWrite(heaterPin0, Output0);

  //    Input1 = T1;
  //    myPID1.Compute();
  //    analogWrite(heaterPin1, Output1);

  //    Input2 = T2;
  //    myPID2.Compute();
  //    analogWrite(heaterPin2, Output2);



  //Varibale for Analog Out Calculation
  int16_t aout_val;

  //get fresh reading
  int16_t in = analogRead(hallPin);

  dia += (((float)convert2dia(in) / 1000.0) - dia) / smooth;

  // Map diameter to DAC output value
  int16_t help_dia_int = (int16_t)(dia * 1000);
  if (doPullerPID) {
    Input3 = help_dia_int;
    //      myPID3.Compute();
  }
  else {
    Output3 = pullerSpeed;
  }
  if (Output3 < maxPullerSpeed) {
    Puller.setSpeed(Output3 * MICROSTEPS);
  }
  else {
    Serial.println("motor over speed check PID value and filament flow");
  }

  // drive motor
  Extruder.setSpeed(extruderSpeed);
  Extruder.runSpeed();
  Puller.runSpeed();

  Serial.print("filament dimator ");
  Serial.print(help_dia_int);
  Serial.print(" heater0 temp = ");
  Serial.print(T0);
  Serial.print(" heater1 temp = ");
  Serial.print(T1);
  Serial.print(" heater2 temp = ");
  Serial.print(T2);
  Serial.println(millis());
  time2 = millis();

  telapsed = time2 - time1;
  delay(500 - telapsed);
}

void doG0() {
  Serial.println(verson);
  Serial.print("smooth = ");
  Serial.println(smooth);
  Serial.print("dia = ");
  Serial.println(dia);
  Serial.print("numReadings = ");
  Serial.println(numReadings);
  Serial.print("Setpoint0 = ");
  Serial.println(Setpoint0);
  Serial.print("Setpoint1 = ");
  Serial.println(Setpoint1);
  Serial.print("Setpoint2 = ");
  Serial.println(Setpoint2);
  Serial.print("P0 = ");
  Serial.print(P0);
  Serial.print(" I0 = ");
  Serial.print(I0);
  Serial.print(" D0 = ");
  Serial.println(D0);
  Serial.print("P1 = ");
  Serial.print(P1);
  Serial.print(" I1 = ");
  Serial.print(I1);
  Serial.print(" D1 = ");
  Serial.println(D1);
  Serial.print("P2 = ");
  Serial.print(P2);
  Serial.print(" I2 = ");
  Serial.print(I2);
  Serial.print(" D2 = ");
  Serial.println(D2);
  Serial.print("mintemp = ");
  Serial.print(mintemp);
  Serial.print(" maxtemp = ");
  Serial.println(maxtemp);
  Serial.print("Setpoint3 = ");
  Serial.println(Setpoint3);
  Serial.print("P3 = ");
  Serial.print(P3);
  Serial.print(" I3 = ");
  Serial.print(I3);
  Serial.print(" D3 = ");
  Serial.println(D3);
  Serial.print(" MICROSTEPS = ");
  Serial.println(MICROSTEPS);
  Serial.print(" runExtruder = ");
  Serial.println(runExtruder);
  Serial.print(" doPullerPID = ");
  Serial.println(doPullerPID);
  Serial.print(" pullerSpeed = ");
  Serial.println(pullerSpeed);
  Serial.print(" extruderSpeed = ");
  Serial.println(extruderSpeed);
}

void doG1() {
  if (runExtruder == 0) {
    digitalWrite(SLEEPe, LOW);
    runExtruder = 1;
  } else {
    digitalWrite(SLEEPe, HIGH);
    runExtruder = 0;
  }
  Serial.println(runExtruder);
}

void doG2() {
  char*       paramStr;
  int         numParams;
  paramValue  theParams[MAX_PARAMS];

  numParams = GCodeParser.numParams();      // How amny params we got in this one?
  for (int i = 0; i < numParams; i++) {     // Loop through all the params..
    paramStr = GCodeParser.getParamBuff(); // We get a parameter. Each time it steps to the next one.
    theParams[i] = parseParam(paramStr);   // This function pulls the parameter apart and hands back the bits.
    free(paramStr);                        // Dump the parameter buffer ASAP.
  }
  Serial.print("With ");
  Serial.print(numParams);
  extruderSpeed = numParams;
}

void doG3() {
  char*       paramStr;
  int         numParams;
  paramValue  theParams[MAX_PARAMS];

  numParams = GCodeParser.numParams();      // How amny params we got in this one?
  for (int i = 0; i < numParams; i++) {     // Loop through all the params..
    paramStr = GCodeParser.getParamBuff(); // We get a parameter. Each time it steps to the next one.
    theParams[i] = parseParam(paramStr);   // This function pulls the parameter apart and hands back the bits.
    free(paramStr);                        // Dump the parameter buffer ASAP.
  }
  Serial.print("With ");
  Serial.print(numParams);
  pullerSpeed = numParams;
  doPullerPID = 0;
}

void doG4() {
  if (doPullerPID == 1) {
    doPullerPID = 0;
  } else {
    doPullerPID = 1;
  }
  Serial.println(doPullerPID);
}

void doG10() {
  char*       paramStr;
  int         numParams;
  paramValue  theParams[MAX_PARAMS];

  numParams = GCodeParser.numParams();      // How amny params we got in this one?
  for (int i = 0; i < numParams; i++) {     // Loop through all the params..
    paramStr = GCodeParser.getParamBuff(); // We get a parameter. Each time it steps to the next one.
    theParams[i] = parseParam(paramStr);   // This function pulls the parameter apart and hands back the bits.
    free(paramStr);                        // Dump the parameter buffer ASAP.
  }
  Serial.print("With ");
  Serial.print(numParams);
  Setpoint0 = numParams;
}

void doG11() {
  char*       paramStr;
  int         numParams;
  paramValue  theParams[MAX_PARAMS];

  numParams = GCodeParser.numParams();      // How amny params we got in this one?
  for (int i = 0; i < numParams; i++) {     // Loop through all the params..
    paramStr = GCodeParser.getParamBuff(); // We get a parameter. Each time it steps to the next one.
    theParams[i] = parseParam(paramStr);   // This function pulls the parameter apart and hands back the bits.
    free(paramStr);                        // Dump the parameter buffer ASAP.
  }
  Serial.print("With ");
  Serial.print(numParams);
  Setpoint1 = numParams;
}

void doG12() {
  char*       paramStr;
  int         numParams;
  paramValue  theParams[MAX_PARAMS];

  numParams = GCodeParser.numParams();      // How amny params we got in this one?
  for (int i = 0; i < numParams; i++) {     // Loop through all the params..
    paramStr = GCodeParser.getParamBuff(); // We get a parameter. Each time it steps to the next one.
    theParams[i] = parseParam(paramStr);   // This function pulls the parameter apart and hands back the bits.
    free(paramStr);                        // Dump the parameter buffer ASAP.
  }
  Serial.print("With ");
  Serial.print(numParams);
  Setpoint2 = numParams;
}

//Convert AD Value to Diameter
int16_t convert2dia(int16_t in) {

  //converts an ADC reading to diameter
  //Inspired by Sprinter / Marlin thermistor reading


  uint8_t i;

  for (i = 1; i < numtemps; i++)
  {
    //check if we've found the appropriate row
    if (dia_table[i][0] > in)
    {
      float slope = ((float)dia_table[i][1] - dia_table[i - 1][1]) / ((float)dia_table[i][0] - dia_table[i - 1][0]);
      float indiff = ((float)in - dia_table[i - 1][0]);
      float outdiff = slope * indiff;
      int16_t out = (int16_t)outdiff + dia_table[i - 1][1];
      return (out);
      break;
    }
  }
}

paramValue parseParam(char* theParam) {

  paramValue  aValue;
  switch (theParam[0]) {
    case 'E' : aValue.theType = puller;      break;
    case 'F' : aValue.theType = feed;      break;
    case 'I' : aValue.theType = letterI;   break;
    case 'J' : aValue.theType = letterJ;   break;
    case 'K' : aValue.theType = letterK;   break;
    default  :
      Serial.println("Unknown param type");
  }
  aValue.theValue = atof(&(theParam[1]));
  return aValue;
}

long ReadthermistorPin0() { /* function smooth */
  ////Perform average on sensor readings
  long average0;
  // subtract the last reading:
  total0 = total0 - readings0[readIndex0];
  // read the sensor:
  readings0[readIndex0] = analogRead(thermistorPin0);
  // add value to total:
  total0 = total0 + readings0[readIndex0];
  // handle index
  readIndex0 = readIndex0 + 1;
  if (readIndex0 >= numReadings) {
    readIndex0 = 0;
  }
  // calculate the average:
  average0 = total0 / numReadings;

  return average0;
}

long ReadthermistorPin1() { /* function smooth */
  ////Perform average on sensor readings
  long average1;
  // subtract the last reading:
  total1 = total1 - readings1[readIndex1];
  // read the sensor:
  readings1[readIndex1] = analogRead(thermistorPin1);
  // add value to total:
  total1 = total1 + readings1[readIndex1];
  // handle index
  readIndex1 = readIndex1 + 1;
  if (readIndex1 >= numReadings) {
    readIndex1 = 0;
  }
  // calculate the average:
  average1 = total1 / numReadings;

  return average1;
}

long ReadthermistorPin2() { /* function smooth */
  ////Perform average on sensor readings
  long average2;
  // subtract the last reading:
  total2 = total2 - readings2[readIndex2];
  // read the sensor:
  readings2[readIndex2] = analogRead(thermistorPin2);
  // add value to total:
  total2 = total2 + readings2[readIndex2];
  // handle index
  readIndex2 = readIndex2 + 1;
  if (readIndex2 >= numReadings) {
    readIndex2 = 0;
  }
  // calculate the average:
  average2 = total2 / numReadings;

  return average2;
}
