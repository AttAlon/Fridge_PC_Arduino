//
//Definitions and globals
//
//HW setup and layout
int TemperaturePin        = A0; // Input pin for the potentiometer
int TempDrivePin          = 5;  // pin for the termostat read

int LedPin                = 13; // pin for the LED
int DoorCheckDrivePin     = 2;  // door drive pin
int DoorCheckReadPinUpper = 3;  // upper door status (normaly close) - BLUE
int DoorCheckReadPinLower = 4;  // lower door status (normly open) - WHITE

int FanPin                = 6;  // Fan - active HIGH
int DefrostPin            = 7;  // Heat - active HIGH
int CoolingPin            = 8;  // Cooling - active HIGH
int AlarmPin              = 9;  // Alarm - active LOW

unsigned long timestamp   = 0;  //a timestamp in secower on delay
int lastTemperature       = 0; //the last temperature read

#define POWER_ON_DELAY_IN_SEC       3*60 //Delay after power ON before cooling can start. default 3 Min
#define DELAY_AFTER_COOLING_STOP    10*60 //Sec delay bafore we can start the cooling after it was stoped. defalt 5min.
#define DOOR_OPEN_STOP_COOLING_TIME 30    //Sec after DOOR is opened to stop colling, default 30 Sec
#define DOOR_OPEN_SOUND_ALERT_TIME  60    //Sec after DOOR is opened to sound ALARM, default 60 Sec

#define HOUR                        60*60    // 1 hour in Sec
#define DEFROST_WAIT_TIME           4*HOUR   //Time in Sec to wait for Defrose action. default 4H
#define DEFROST_ON_TIME             15*60    //Time in Sec to activate Defrost. default 15 min
#define MAX_COMPRESOR_ON_TIME       2*HOUR   //Time in Sec to allow compressor ON, 
                                             // when reached compressor will be switch OFF
                                             // default is 2H
#define LOOP_SLEEP_MSEC             1000      //main loop sleep time in mSec. defult is 1 Sec
#define DRIVE_STABILIZE             20
#define TEMP_READ_LOOP              4         //number of time to read temperatur for average

//temperature reading - temperature reading is ranging from 0 to 1023 whar 0 is hot and 1023 is coolest.
#define TEMP_TARGET_LOW             450       //target temperature for cooling, when reach cooling is stoped. range 1-1023
#define TEMP_KICK_OFF               390       //bottom temperature to start cooling again.

//Globals

int doorOpenCount = 0;          //loop count for door open
int defrostWaitCount = 0;       //defrost wait loop counter, continue to count also when defrost is ON
int defrostOnCount = 0;         //defrost on loop count, when 0 defrost is OFF
int timeAfterCoolingStoped = DELAY_AFTER_COOLING_STOP; // the counter after cooling was stoped
int puwerOnDelay = POWER_ON_DELAY_IN_SEC;
int compresorOnCount = 0;       //loop count when compresor is ON
//
//  Functions definitiosn
//

void doorCheck ();
bool isDoorOpen ();           
void doorOpenLedOn ();
void doorOpenLedOff ();

void defrostCheck ();
void defrostOn ();             
void defrostOff ();            

void coolCheck ();
int readTemp ();               

void stopCooling ();              
void startCooling();              
void stopFan();                
void startFan();               

void setup()
{
  // declare the ledPin as an OUTPUT:
  pinMode(LedPin, OUTPUT);
  pinMode(DoorCheckDrivePin, OUTPUT);
  pinMode(DoorCheckReadPinUpper, INPUT);
  pinMode(DoorCheckReadPinLower, INPUT);
  pinMode(TempDrivePin, OUTPUT);
  pinMode(FanPin, OUTPUT);
  pinMode(DefrostPin, OUTPUT);
  pinMode(CoolingPin, OUTPUT);
  pinMode(AlarmPin, OUTPUT);

  digitalWrite(FanPin, LOW);
  digitalWrite(DefrostPin, LOW);
  digitalWrite(CoolingPin, LOW);
  digitalWrite(AlarmPin, HIGH);

  Serial.begin(115200);

  Serial.println("Start V1.2");

}
void logHeader() {
  long H = timestamp / (HOUR);
  long M = (timestamp / 60) % 60;
  long S = timestamp % 60;
  if (H < 10) {
    Serial.print("0");
  }
  Serial.print(H);
  Serial.print(":");  
  if (M < 10) {
    Serial.print("0");
  }
  Serial.print(M);
  Serial.print(":");
  if (S < 10) {
    Serial.print("0");
  }
  Serial.print(S);
  Serial.print("\t");
}
void log(String msg) {
  logHeader();
  Serial.println(msg);

}
byte isAlarm = 0;
byte alarmTogale = 0;

void alarmCheck() {
  if(isAlarm) {
    alarmTogale = !alarmTogale;
    if (alarmTogale==0) {
          digitalWrite(AlarmPin, LOW);

    } else {
          digitalWrite(AlarmPin, HIGH);

    }
  } else {
    digitalWrite(AlarmPin, HIGH);
  }
  
}

//SondAlert
#define ALARM_ON  1
#define ALARM_OFF 0
void setAlarm(byte value){
  if (isAlarm != value) {
    log("****** Set ALARM ******", value);
  }
  isAlarm = value;
  
}
void log(String msg, long data) {
  logHeader();
  Serial.print(msg);
  Serial.print(" ");
  Serial.println(data);

}

void loop()
{
  timestamp++;

  doorCheck();

  defrostCheck ();
  if (puwerOnDelay <= 0) {
    coolCheck ();
  } else {
    puwerOnDelay--;
    if (puwerOnDelay == 0) {
      log("Power on delay is finised");
    }
    readTemp();
  }
  alarmCheck();
  
  delay(LOOP_SLEEP_MSEC - ((TEMP_READ_LOOP+1)*DRIVE_STABILIZE)); // wait for a second

  /*
  digitalWrite(LedPin, HIGH);
  delay(LOOP_SLEEP_MSEC - ((TEMP_READ_LOOP+1)*DRIVE_STABILIZE)); // wait for a second
  digitalWrite(LedPin, LOW);
  delay(500); // DEBUG LED BLINK!!!
  */
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//
// doorCheck - check door status and operate accordingly.
//
void doorCheck ()
{
  if (isDoorOpen())
  {
    doorOpenLedOn();
    doorOpenCount++;
    
  }
  else
  {
    if (doorOpenCount > 0) {
      doorOpenLedOff ();
      doorOpenCount = 0;
    }
  }
}

//
//isDoorOpen - return true if any of the doors are open.
//  upper door switch is NC and the read for open is 1
//  lower door switch is NO and the read for open is 0
//  We use the DoorCheckDrivePin do drive power to the pullup resistor to save power.
//    
bool isDoorOpen ()
{
  //set drive pin
  digitalWrite(DoorCheckDrivePin, HIGH);
  delay(DRIVE_STABILIZE);
  int upper = digitalRead(DoorCheckReadPinUpper);
  int lower= digitalRead(DoorCheckReadPinLower);

  //DEBUG
  /*
  if (doorOpenCount > 0) {
      Serial.print("doorOpenCount = ");
      Serial.print(doorOpenCount);
      Serial.print("  Upper = ");
      Serial.print(upper);
      Serial.print("  Lower = ");
      Serial.println(lower);
  }
  if ((upper == 1) && (doorOpenCount == 0)) {
    Serial.println("Upper door is OPEN");
  }
  if ((lower == 0) && (doorOpenCount == 0)) {
    Serial.println("Lower door is OPEN");

  }
  */
  //END DEBUG
  
  //reset drive pin  
  digitalWrite(DoorCheckDrivePin, LOW);
  
  return ((upper == 1) || (lower == 0));
}
    
//
//  doorOpenLedOn - set led on (not implemented yet) 
//      and update cooling fan and alert when needed base on time windows.
//
void doorOpenLedOn ()
{
      
  //DOOR LED ON
  log("Door is OPENED");
  stopFan();
  //check stop cooling time
  if (DOOR_OPEN_STOP_COOLING_TIME <= doorOpenCount)
  {
    //stop coolong
    stopCooling();
    
  }

  //check sound alert time
  if(DOOR_OPEN_SOUND_ALERT_TIME <= doorOpenCount)
  {
    //start the sond alert
    setAlarm(ALARM_ON); 

  }
}

//
// doorOpenLedOff - door is closed, continue the Fan and stop the alarm.
// 
void doorOpenLedOff ()
{
      
  //DOOR LED OFF
  log("Door is CLOSED");
  //return the fan
  startFan ();
  
  //stop the alert
  setAlarm(ALARM_OFF); 
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//
// DefrostCheck - we do defrost at a predefined intervals of DEFROST_WAIT_TIME for a period of DEFROST_ON_TIME
//
void defrostCheck ()
{
  if (DEFROST_WAIT_TIME <= defrostWaitCount)
  {
    //start the defrost
    if (DEFROST_WAIT_TIME == defrostWaitCount) {

      defrostOn();
    }
    defrostOnCount++;
  }

  if (DEFROST_ON_TIME < defrostOnCount)
  {
    //stop the defrost
    defrostOff();

    //reset all the counters
    defrostWaitCount = 0;
    defrostOnCount = 0;
  }
  
  defrostWaitCount++;
}

//
// defrostOn - start defrost and stop coolling and fan
//
void defrostOn ()
{
  //start the defrost
  log("Start Defrost");    
  stopCooling();
  stopFan();
  digitalWrite(DefrostPin, HIGH);
}

// 
// defrostOff- stop defrost and start the Fan
//
void defrostOff ()
{
  digitalWrite(DefrostPin, LOW);

  //stop the defrost
  log("Stop Defrost");    
  startCooling();
  startFan();

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//coolCheck - start stop cooling base on the current temperature and cooling max time limit: MAX_COMPRESOR_ON_TIME
//
void coolCheck()
{
  // no need to start cooling if door is open or defrost is ON
  if (doorOpenCount == 0 && defrostOnCount == 0)
  {
    int currentTemp = readTemp();
    if (compresorOnCount > 0) {
      //  ON
      compresorOnCount++;
      if (currentTemp > TEMP_TARGET_LOW) {
        //target reached, we can stop
        stopCooling();
        
      }
    } else {
      //OFF
      timeAfterCoolingStoped ++;
      //we can start only if we stopend before DELAY_AFTER_COOLING_STOP
      if (timeAfterCoolingStoped > DELAY_AFTER_COOLING_STOP) {
        
        
        if (currentTemp < TEMP_KICK_OFF) {
          //its too hot, we can start cooling again
          startCooling();
          startFan();
          
        }
      }
    }

    if (MAX_COMPRESOR_ON_TIME < compresorOnCount) {
      log("compresor on for too long");
      stopCooling();
      compresorOnCount = 0;  //we will start when temp is below TEMP_KICK_OFF and time after stop is above DELAY_AFTER_COOLING_STOP
    }

  }
}  
int readTemp() {
  
  if (TEMP_READ_LOOP == 0){
    return 0;
  }
  //set drive pin
  digitalWrite(TempDrivePin, HIGH);
  
  int loop = 0;
  long sum = 0;
  for(loop =0; loop < TEMP_READ_LOOP; loop++) {
    delay(DRIVE_STABILIZE);
    sum = sum +  analogRead(TemperaturePin);
  }
  digitalWrite(TempDrivePin, LOW);
  int currentRead = sum / loop;
  if ((lastTemperature > (currentRead+1)) || (lastTemperature < (currentRead-1))) {
    
    log("Temp=", currentRead);
    lastTemperature = currentRead;
  }  
  return currentRead;
  
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//cool
void  stopCooling ()
{
  log("Stop Cooling");
  digitalWrite(CoolingPin, LOW);
  timeAfterCoolingStoped = 0;
  compresorOnCount = 0;
}

void  startCooling()
{
  log("Start Cooling");
  digitalWrite(CoolingPin, HIGH);
  compresorOnCount = 1;
}
//cool

//fan
void stopFan()           
{
  log("Stop FAN");   
  digitalWrite(FanPin, LOW);  
 
}

void startFan()
{
  log("Start FAN"); 
  digitalWrite(FanPin, HIGH);  
 
}
//fan
