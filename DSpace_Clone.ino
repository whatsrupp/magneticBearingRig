#include <FilterOnePole.h>
#include <PID_v1.h>
#include <MCPDAC.h>

const int SENSOR_A_PIN = A5;
const int SENSOR_B_PIN = A4;
const int POTENTIOMETER_PIN = A3;
const int PUSH_BUTTON_PIN = 8;

double  brownDemand;
double yellowDemand;
double blueDemand;
double redDemand;

double y;
double x;

double filterFrequency = 80;

float sensorAOut;
float sensorBOut;

FilterOnePole LowpassFilter(LOWPASS, filterFrequency); 

double pidAInput, pidAOutput, pidASetpoint;
double KpA = 1, KiA=0, KdA =0;
PID PidA(&pidAInput, &pidAOutput, &pidASetpoint, KpA, KiA, KdA, DIRECT);


void initialisePidA(){
  pidASetpoint = 0;
  PidA.SetOutputLimits(-2000, 2000);
  PidA.SetSampleTime(2);
  PidA.SetMode(AUTOMATIC);
}

double pidBInput, pidBOutput, pidBSetpoint;
double KpB = 1, KiB=0, KdB =0;
PID PidB(&pidBInput, &pidBOutput, &pidBSetpoint, KpB, KiB, KdB, DIRECT);

void initialisePidB(){
  
  pidBSetpoint = 0;
  PidB.SetOutputLimits(-2000, 2000);
  PidB.SetSampleTime(2);
  PidB.SetMode(AUTOMATIC);
}

const int DAC_A1 = A0;
const int DAC_A2 = A1;
const int DAC_B1 = A2;
const int DAC_B2 = A3;

const int SS_PIN_A = 10;
const int SS_PIN_B = 9;
MCPDACClass MCPDAC_A, MCPDAC_B;

void setUpDACConnection(MCPDACClass& DAC_CLASS, int slavePinNumber){
   DAC_CLASS.begin(slavePinNumber);
   DAC_CLASS.setGain(CHANNEL_A, GAIN_HIGH);
   DAC_CLASS.setGain(CHANNEL_B, GAIN_HIGH);
}

void initialiseDACs(){
  setUpDACConnection(MCPDAC_A, SS_PIN_A);
  setUpDACConnection(MCPDAC_B, SS_PIN_B);
}

void openSerialConnection(){
  Serial.begin(115200);  
}

void initialiseButton(){
  pinMode(PUSH_BUTTON_PIN, INPUT);
}





void setup() {

  while(!isButtonPressed()){
    
  }
  openSerialConnection();
  initialisePidA();
  initialisePidB();
  initialiseDACs();
  initialiseButton();
}

void updatePids(){
  pidAInput = y;
  pidBInput = x;
  double p = 1325;
  double i = 1400;
  double d = 6.5;
  
  PidA.SetTunings(p,i,d);
  PidB.SetTunings(p,i,d);

  PidA.Compute();
  PidB.Compute();
}

float analogToVolts(float signal){
  return signal * (5.0/ 1023.0);
}

double analyseSensorInput(float signal){
  LowpassFilter.input(signal);
  double filteredSignal = LowpassFilter.output();
  double voltage = analogToVolts(signal);
  return voltage;
}

void convertDisplacementsToXandY(){
  double sensorASetpoint = 3.4;
  double sensorBSetpoint = 3.6;
  double yBar = sensorASetpoint - sensorAOut;
  double xBar = sensorBSetpoint - sensorBOut;
  double alpha = atan2(yBar,xBar);
  double magnitude = sqrt(pow(xBar,2)+pow(yBar,2));
  const double piOver4= HALF_PI/2;
  double adjustmentAngle = alpha + piOver4;

   x = cos(adjustmentAngle)*magnitude;
   y = sin(adjustmentAngle)*magnitude;
}



void processNewSensorSignals(){
  sensorAOut = analyseSensorInput(analogRead(SENSOR_A_PIN));
  sensorBOut = analyseSensorInput(analogRead(SENSOR_B_PIN));
}

double getGainMultiplier(){
    double reading = analogToVolts(analogRead(POTENTIOMETER_PIN));
//    Serial.print(reading);
//    Serial.print(" ");
  return (reading/(5))*-1 + 1;
}

bool isButtonPressed(){
    int val = digitalRead(PUSH_BUTTON_PIN);
    bool isPressed = val == 0;
    return isPressed;
}

double getOverallGain(){
  if(isButtonPressed()){
    return 1;
  }
  else{
    return 0;
  }
}





double voltsToMilliVolts(double volts){
 return volts*1000.00; 
}

double computeControlDemandA(){
  return (pidBOutput - pidAOutput )/sqrt(2);
}

double computeControlDemandB(){
   return (pidAOutput + pidBOutput)/(-1*sqrt(2));
}

void checkLoopSpeed(){
    
  static int counter = 0;
  int demand = 0;
  if(counter%2==0){
   demand = 4000;
  }else{
    demand = 0;
  }
  counter += 1;
  MCPDAC_A.setVoltage(CHANNEL_A,demand); //Amplifier 6
}

void sendDemandVoltageToDACs(){
//  checkLoopSpeed();

  static int counter = 0;
  int demand = 0;
  if(counter%2==0){
   demand = 4000;
  }else{
    demand = 0;
  }
  counter += 1;
  
  double biasDemand = 2000;
  double lowerBiasDemand = biasDemand;
  double upperBiasDemand = biasDemand;
  
  double controlDemandA = computeControlDemandA();
  double controlDemandB = computeControlDemandB();
//  const int threshold = 1280;
//  double controlDemandA = getGainMultiplier()*(1750-threshold)+(threshold);
//    double controlDemandA = getGainMultiplier()*4000-2000;

//  double controlDemandB = getGainMultiplier()*4000-2000;
//  Serial.print(controlDemandA);
//  Serial.print(" ");
//  Serial.print(controlDemandB);
//  Serial.print(" ");

  double overallGain = getOverallGain();
//  Serial.print(overallGain);
  double brownBias = 1300;
  double yellowBias = 2000;
  double blueBias = 1350;
  double redBias = 1850;
  brownDemand= (brownBias + controlDemandB); //Pair A Top
  yellowDemand= (yellowBias - controlDemandB); //Pair A Bottom
  blueDemand = (blueBias + controlDemandA); //Pair B Top
  redDemand = (redBias - controlDemandA); //Pair B Bottom

  if(brownDemand<0){
    brownDemand = 0;
  }
    if(yellowDemand<0){
    yellowDemand = 0;
  }
    if(blueDemand<0){
    blueDemand = 0;
  }
    if(redDemand<0){
    redDemand = 0;
  }

  if(brownDemand>4095){
    brownDemand = 4095;
  }

  if(yellowDemand>4095){
    yellowDemand = 4095;
  }
  if(blueDemand>4095){
    blueDemand = 4095;
  }
  if(redDemand>4095){
    redDemand = 4095;
  }

  
  
  double amplifierGain = 2.5;
//
//  Serial.print(x);
//  Serial.print(" ");
//   Serial.print(y);
//  Serial.print(" ");
  
//  Serial.print ((brownDemand/1000.00) *amplifierGain*overallGain);
//  Serial.print(" ");
//  Serial.print (yellowDemand/1000.00 * amplifierGain*overallGain);
//  Serial.print(" ");
//  Serial.print (blueDemand/1000.00 * amplifierGain*overallGain);
//  Serial.print(" ");
//  Serial.print (redDemand/ 1000.00 * amplifierGain*overallGain);
//  Serial.print(" ");

//  Serial.print(brownDemand);
//  Serial.print(" ");
//  Serial.print (blueDemand);
//  Serial.print(" ");
//  Serial.print( yellowDemand);
//  Serial.print(" ");
//  Serial.print (redDemand);
//  Serial.print(" ");
//    Serial.print(pidAOutput);
//  Serial.print(" ");
//    Serial.print(pidBOutput);
//  Serial.print(" ");

  MCPDAC_A.setVoltage(CHANNEL_A,brownDemand*overallGain); //Amplifier 6
  MCPDAC_A.setVoltage(CHANNEL_B,blueDemand*overallGain); //Amplifier 7
  MCPDAC_B.setVoltage(CHANNEL_A,yellowDemand*overallGain); //Amplifier 8
  MCPDAC_B.setVoltage(CHANNEL_B,redDemand*overallGain); //Amplifier 9

//  MCPDAC_A.setVoltage(CHANNEL_A,0); //Brown
//  MCPDAC_A.setVoltage(CHANNEL_B, 0); //Blue
//  MCPDAC_B.setVoltage(CHANNEL_A,0); //Yellow
//  MCPDAC_B.setVoltage(CHANNEL_B,demand); //Red
}

void(* resetFunc) (void) = 0;

void loop() {

  if(!isButtonPressed()){
    resetFunc();
  }
  processNewSensorSignals();
  convertDisplacementsToXandY();
  updatePids();
  sendDemandVoltageToDACs();
  printVariables();
}

void printVariables(){
  printSensorReadings();
  printTimeElapsed();
  printPidAValues();
  printPidBValues();
  printDacOutputs();
  endPrintSequence();
}


void printDacOutputs(){
  float dacA1Out = analogRead(DAC_A1);
  float dacA2Out = analogRead(DAC_A2);
  float dacB1Out = analogRead(DAC_B1);
  float dacB2Out = analogRead(DAC_B2);

  float dacA1Voltage = analogToVolts(dacA1Out); 
  float dacA2Voltage = analogToVolts(dacA2Out); 
  float dacB1Voltage = analogToVolts(dacB1Out); 
  float dacB2Voltage = analogToVolts(dacB2Out); 
  Serial.print(dacA1Voltage);
  Serial.print(" ");
  Serial.print(dacA2Voltage);
  Serial.print(" ");
  Serial.print(dacB1Voltage);
  Serial.print(" ");
  Serial.print(dacB2Voltage);
  Serial.print(" ");
}

void printSensorReadings(){
  Serial.print(sensorAOut);
  Serial.print(" ");
  Serial.print(sensorBOut);
  Serial.print(" ");

}
void printPidBValues(){

  Serial.print(PidB.GetKp());
  Serial.print(" ");
  Serial.print(PidB.GetKi());
  Serial.print(" ");
  Serial.print(PidB.GetKd());
  Serial.print(" ");
    Serial.print(pidBOutput);
  Serial.print(" ");
  Serial.print(pidBSetpoint);
  Serial.print(" ");
}
void printPidAValues(){
  Serial.print(PidA.GetKp());
  Serial.print(" ");
  Serial.print(PidA.GetKi());
  Serial.print(" ");
  Serial.print(PidA.GetKd());
  Serial.print(" ");
  Serial.print(pidAOutput);
  Serial.print(" ");
  Serial.print(pidASetpoint);
  Serial.print(" ");  
}
void printTimeElapsed(){
  Serial.print(millis());
  Serial.print(" ");
}

void endPrintSequence(){
  Serial.println(" ");
}
