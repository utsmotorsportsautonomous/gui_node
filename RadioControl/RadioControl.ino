#define PPM_DATA_PIN 2
#define PPM_CHANNELS 10
#define MANUAL_RC_ENABLED // Turns on Module

#define MANUAL_RC_ON_BOARD_ENABLE_PIN 5

#define MANUAL_RC_STEERING_CH  4
#define MANUAL_RC_THROTTLE_CH 2
#define MANUAL_RC_MODE_AND_E_STOP_KILL_CH 5  

//#define MANUAL_RC_BOOST_SWITCH_CH 8

#define MANUAL_RC_RECEIVER_MAX_PERIOD 2000
#define MANUAL_RC_RECEIVER_MIN_PERIOD 1000

#define MANUAL_RC_SWITCH_STATE_UP_LIMIT 1000
#define MANUAL_RC_SWITCH_STATE_DOWN_LIMIT 1750
#include <PinChangeInterrupt.h>

#ifdef MANUAL_RC_ENABLED

#include <PPMReader.h>


unsigned long SteeringTimer;
unsigned long ThrottleTimer;
unsigned long ActivateTimer;

unsigned long SteeringPeriod;
unsigned long ThrottlePeriod;
unsigned long ActivatePeriod;

unsigned int PPMPeriods[PPM_CHANNELS + 1];

struct RCControlModel
{
  unsigned char switch1State;
  unsigned int switch1Period;
  unsigned int steeringPeriod;
  unsigned int throttlePeriod;
  unsigned int panPeriod;
  unsigned int tiltPeriod;
};

RCControlModel RCController1;

PPMReader ppm(PPM_DATA_PIN, PPM_CHANNELS);

//Servo PanServo;
//Servo TiltServo;


void PPMUpdater()
{
  for (int i = 0; i < PPM_CHANNELS; i++)
  {
    PPMPeriods[i] = ppm.latestValidChannelValue(i,0);
   // dataContainer.PPMChannels[i] = PPMPeriods[i];
    //Serial.print(PPMPeriods[i]);
    //Serial.print(',');
    
  }
  //Serial.println("");
  //Serial.println(PPMPeriods[3]);
  //Serial.println(PPMPeriods[1]);
  //Serial.println(PPMPeriods[5]);
  RCController1.steeringPeriod = PPMPeriods[MANUAL_RC_STEERING_CH];
  RCController1.throttlePeriod = PPMPeriods[MANUAL_RC_THROTTLE_CH];
  RCController1.switch1Period = PPMPeriods[MANUAL_RC_MODE_AND_E_STOP_KILL_CH];

  if (RCController1.switch1Period <= MANUAL_RC_SWITCH_STATE_UP_LIMIT)
  {
    RCController1.switch1State = 2;
  }
  else if (RCController1.switch1Period > MANUAL_RC_SWITCH_STATE_UP_LIMIT && RCController1.switch1Period < MANUAL_RC_SWITCH_STATE_DOWN_LIMIT)
  {
    RCController1.switch1State = 1;
  }
  else
  {
    RCController1.switch1State = 0;
  }
}



void ManualControlSetup()
{
  //pinMode(MANUAL_RC_MODE_AND_E_STOP_KILL_PIN, INPUT_PULLUP);
  //pinMode(MANUAL_RC_STEERING_PIN, INPUT_PULLUP);
  //pinMode(MANUAL_RC_THROTTLE_PIN, INPUT_PULLUP);
  pinMode(MANUAL_RC_ON_BOARD_ENABLE_PIN, INPUT_PULLUP);
}

void ManualControlProcessor()
{
  /*DroidCar.steeringAngle = map(RCController1.steeringPeriod, MANUAL_RC_RECEIVER_MIN_PERIOD, MANUAL_RC_RECEIVER_MAX_PERIOD, 0, 180);


  if (RCController1.throttlePeriod > (MANUAL_RC_RECEIVER_MIN_PERIOD + MANUAL_RC_RECEIVER_MAX_PERIOD) / 2)
  {
    DroidCar.travelDirection = DroidCar.Forward;
    DroidCar.throttle = constrain(map(RCController1.throttlePeriod, (MANUAL_RC_RECEIVER_MIN_PERIOD + MANUAL_RC_RECEIVER_MAX_PERIOD) / 2, MANUAL_RC_RECEIVER_MAX_PERIOD, 0, 255), 0, 255);
  }
  else if (RCController1.throttlePeriod < (MANUAL_RC_RECEIVER_MIN_PERIOD + MANUAL_RC_RECEIVER_MAX_PERIOD) / 2)
  {
    DroidCar.travelDirection = DroidCar.Backward;
    DroidCar.throttle = constrain(map(RCController1.throttlePeriod, MANUAL_RC_RECEIVER_MIN_PERIOD, (MANUAL_RC_RECEIVER_MIN_PERIOD + MANUAL_RC_RECEIVER_MAX_PERIOD) / 2, 255, 0), 0, 255);
  }
  else
  {
    DroidCar.travelDirection = DroidCar.Neutral;
    DroidCar.throttle = 0;
  }*/
}


#endif // MANUAL_RC_ENABLED

#include <ArduinoJson.h>  // Using arduinoJSON version 6
StaticJsonDocument  <256> doc;
char json[] = "{\"Throttle\":\"100\",\"Steering\":125,\"Mode\":0,\"Stop\":0}";


void setup(){
  Serial.begin(9600);
  //attachInterrupt(digitalPinToInterrupt(3), ManualControlThrottleInputInterrupt, CHANGE);
  auto error = deserializeJson(doc, json);
  if (error) {
      Serial.print(F("deserializeJson() failed with code "));
      Serial.println(error.c_str());
      return;
  }
  else{
    Serial.println("JSON deserialize SUCCESSFUL");
  }
}

void loop(){
  #ifdef MANUAL_RC_ENABLED
    PPMUpdater();
  #endif // MANUAL_RC_ENABLED

  doc["Throttle"] = map(PPMPeriods[3], 1000, 2000, 0, 255);
  doc["Steering"] = map(PPMPeriods[1], 1000, 2000, 0, 255);
  if(PPMPeriods[5] > 1700){
    doc["Mode"] = 2;
  }
  else if(PPMPeriods[5] > 1100){
    doc["Mode"] = 1;
  }
  else{
    doc["Mode"] = 0;
  }
  //doc["Mode"] = PPMPeriods[5];
  if(Serial.available()){ 
    //Serial.println(inByte); 

    //Serial.println(inByte); 
    serializeJson(doc, Serial);  
    Serial.println(""); 


  }
  delay(10);
   
}


void ManualControlThrottleInputInterrupt()
{
  //ManualControlTimerInterrupt(&ThrottleTimer, &ThrottlePeriod, digitalRead(3));
  RCController1.throttlePeriod = ThrottlePeriod;
}


/*void ManualControlTimerInterrupt(unsigned long *TimerVariable, unsigned long *FinalPeriod, bool State)
{
  if (State == true)
  {
    *TimerVariable = micros();
  }
  else
  {
    *FinalPeriod = micros() - *TimerVariable;
  }
}*/
