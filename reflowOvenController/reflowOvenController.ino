/*******************************************************************************
* Title: Reflow Oven Controller
* Version: 1.20
* Date: 26-11-2012
* Company: Rocket Scream Electronics
* Author: Lim Phang Moh
* Website: www.rocketscream.com
* 
* Brief
* =====
* This is an example firmware for our Arduino compatible reflow oven controller. 
* The reflow curve used in this firmware is meant for lead-free profile 
* (it's even easier for leaded process!). You'll need to use the MAX31855 
* library for Arduino if you are having a shield of v1.60 & above which can be 
* downloaded from our GitHub repository. Please check our wiki 
* (www.rocketscream.com/wiki) for more information on using this piece of code 
* together with the reflow oven controller shield. 
*
* Temperature (Degree Celcius)                 Magic Happens Here!
* 245-|                                               x  x  
*     |                                            x        x
*     |                                         x              x
*     |                                      x                    x
* 200-|                                   x                          x
*     |                              x    |                          |   x   
*     |                         x         |                          |       x
*     |                    x              |                          |
* 150-|               x                   |                          |
*     |             x |                   |                          |
*     |           x   |                   |                          | 
*     |         x     |                   |                          | 
*     |       x       |                   |                          | 
*     |     x         |                   |                          |
*     |   x           |                   |                          |
* 30 -| x             |                   |                          |
*     |<  60 - 90 s  >|<    90 - 120 s   >|<       90 - 120 s       >|
*     | Preheat Stage |   Soaking Stage   |       Reflow Stage       | Cool
*  0  |_ _ _ _ _ _ _ _|_ _ _ _ _ _ _ _ _ _|_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
*                                                                Time (Seconds)
*
* This firmware owed very much on the works of other talented individuals as
* follows:
* ==========================================
* Brett Beauregard (www.brettbeauregard.com)
* ==========================================
* Author of Arduino PID library. On top of providing industry standard PID 
* implementation, he gave a lot of help in making this reflow oven controller 
* possible using his awesome library.
*
* ==========================================
* Limor Fried of Adafruit (www.adafruit.com)
* ==========================================
* Author of Arduino MAX6675 library. Adafruit has been the source of tonnes of
* tutorials, examples, and libraries for everyone to learn.
*
* Disclaimer
* ==========
* Dealing with high voltage is a very dangerous act! Please make sure you know
* what you are dealing with and have proper knowledge before hand. Your use of 
* any information or materials on this reflow oven controller is entirely at 
* your own risk, for which we shall not be liable. 
*
* Licences
* ========
* This reflow oven controller hardware and firmware are released under the 
* Creative Commons Share Alike v3.0 license
* http://creativecommons.org/licenses/by-sa/3.0/ 
* You are free to take this piece of code, use it and modify it. 
* All we ask is attribution including the supporting libraries used in this 
* firmware. 
*
* Required Libraries
* ==================
* - Arduino PID Library: 
*   >> https://github.com/br3ttb/Arduino-PID-Library
* - MAX31855 Library (for board v1.60 & above): 
*   >> https://github.com/rocketscream/MAX31855
* - MAX6675 Library (for board v1.50 & below):
*   >> https://github.com/adafruit/MAX6675-library
*
* Revision  Description
* ========  ===========
* 1.20			Adds supports for v1.60 (and above) of Reflow Oven Controller 
*           Shield:
*					  - Uses MAX31855KASA+ chip and pin reassign (allowing A4 & A5 (I2C)
*             to be used for user application).
*					  - Uses analog based switch (allowing D2 & D3 to be used for user 
*						  application).	
*						Adds waiting state when temperature too hot to start reflow process.
*						Corrected thermocouple disconnect error interpretation (MAX6675).
* 1.10      Arduino IDE 1.0 compatible.
* 1.00      Initial public release.
*******************************************************************************/
// Comment either one the following #define to select your board revision
// Newer board version starts from v1.60 using MAX31855KASA+ chip 
//#define  USE_MAX31855
// Older board version below version v1.60 using MAX6675ISA+ chip
#define USE_MAX6675

// ***** INCLUDES *****
#ifdef	USE_MAX31855
	#include <MAX31855.h>
#else
	#include <max6675.h>
#endif
#include <PID_v1.h>

// ***** TYPE DEFINITIONS *****
typedef enum REFLOW_STATE
{
  REFLOW_STATE_IDLE,
  REFLOW_STATE_PREHEAT,
  REFLOW_STATE_SOAK,
  REFLOW_STATE_REFLOW,
  REFLOW_STATE_COOL,
  REFLOW_STATE_TOO_HOT,
  REFLOW_STATE_ERROR
} reflowState_t;

typedef enum REFLOW_STATUS
{
  REFLOW_STATUS_OFF,
  REFLOW_STATUS_ON
} reflowStatus_t;

// ***** CONSTANTS *****
#define TEMPERATURE_ROOM 70
#define TEMPERATURE_SOAK_MIN 120
#define TEMPERATURE_SOAK_MAX 170
#define TEMPERATURE_REFLOW_MAX 220
#define TEMPERATURE_COOL_MIN 100
#define SENSOR_SAMPLING_TIME 1000
#define SOAK_TEMPERATURE_STEP 0.5
#define SOAK_MICRO_PERIOD 1000
#define SSR_PWM_PERIOD 500

// ***** PID PARAMETERS *****
// ***** PRE-HEAT STAGE *****
#define PID_KP_PREHEAT 100
#define PID_KI_PREHEAT 0.05
#define PID_KD_PREHEAT 900
// ***** SOAKING STAGE *****
#define PID_KP_SOAK 200
#define PID_KI_SOAK 0.05
#define PID_KD_SOAK 450
// ***** REFLOW STAGE *****
#define PID_KP_REFLOW 300
#define PID_KI_REFLOW 0.05
#define PID_KD_REFLOW 350
#define PID_SAMPLE_TIME 1000

const char* reflowStatusName[] = {
  "Ready",
  "Pre-heat",
  "Soak",
  "Reflow",
  "Cool",
  "Wait,hot",
  "Error"
};


// ***** PIN ASSIGNMENT *****
#define LED_PIN LED_BUILTIN
#define SSR_PIN 12
#define THERMOCOUPLE_GND_PIN 2
#define THERMOCOUPLE_VCC_PIN 3
#define THERMOCOUPLE_CS_PIN 5
#define THERMOCOUPLE_CLK_PIN 4
#define THERMOCOUPLE_SO_PIN 6

// ***** PID CONTROL VARIABLES *****
double setpoint;
double input;
double prevInput;
double output;

unsigned long windowStartTime;
unsigned long nextCheck;
unsigned long nextRead;
unsigned long timerSoak;
// Reflow oven controller state machine state variable
reflowState_t reflowState;
// Reflow oven controller status
reflowStatus_t reflowStatus;
// Seconds timer
int timerSeconds;
boolean startRequested = false;

// Specify PID control interface
PID reflowOvenPID(&input, &output, &setpoint, PID_KP_PREHEAT, PID_KI_PREHEAT, PID_KD_PREHEAT, DIRECT);

// Specify thermocouple interface
#ifdef	USE_MAX31855
	MAX31855 thermocouple(THERMOCOUPLE_SO_PIN, THERMOCOUPLE_CS_PIN, THERMOCOUPLE_CLK_PIN);
#else
	MAX6675 thermocouple(THERMOCOUPLE_CLK_PIN, THERMOCOUPLE_CS_PIN, THERMOCOUPLE_SO_PIN);
#endif

void setup()
{
  // SSR pin initialization to ensure reflow oven is off
  digitalWrite(SSR_PIN, LOW);
  pinMode(SSR_PIN, OUTPUT);

  // Thermocouple initialization
  digitalWrite(THERMOCOUPLE_GND_PIN, LOW);
  digitalWrite(THERMOCOUPLE_VCC_PIN, HIGH);
  pinMode(THERMOCOUPLE_GND_PIN, OUTPUT);
  pinMode(THERMOCOUPLE_VCC_PIN, OUTPUT);
  
  // LED pins initialization and turn off upon start-up
  digitalWrite(LED_PIN, LOW);
  pinMode(LED_PIN, OUTPUT);

  // Serial communication at 57600 bps
  Serial.begin(57600);

  // Initialize time keeping variable
  nextCheck = millis();
  // Initialize thermocouple reading variable
  nextRead = millis();

  #ifdef USE_MAX31855
    input = thermocouple.readThermocouple(CELSIUS);
  #else
    input = thermocouple.readCelsius();
  #endif
}

void loop()
{
  // Current time
  unsigned long now;

  // Time to read thermocouple?
  if (millis() > nextRead)
  {
    // Read thermocouple next sampling period
    nextRead += SENSOR_SAMPLING_TIME;
    prevInput = input;
    // Read current temperature
		#ifdef	USE_MAX31855
			input = thermocouple.readThermocouple(CELSIUS);
		#else
			input = thermocouple.readCelsius();
		#endif
		
    // If thermocouple problem detected
		#ifdef	USE_MAX6675
			if (isnan(input))
		#else
			if((input == FAULT_OPEN) || (input == FAULT_SHORT_GND) || 
				 (input == FAULT_SHORT_VCC))
    #endif
		{
      // Illegal operation
      reflowState = REFLOW_STATE_ERROR;
      reflowStatus = REFLOW_STATUS_OFF;
    }
  }

  if (millis() > nextCheck)
  {
    // Check input in the next seconds
    nextCheck += 1000;
    // If reflow process is on going
    if (reflowStatus == REFLOW_STATUS_ON)
    {
      // Toggle red LED as system heart beat
      digitalWrite(LED_PIN, !(digitalRead(LED_PIN)));
      // Increase seconds timer for reflow curve analysis
      timerSeconds++;
      // Send temperature and time stamp to serial 
      Serial.print(timerSeconds);
      Serial.print(" ");
      Serial.print(setpoint);
      Serial.print(" ");
      Serial.print(input);
      Serial.print(" ");
      Serial.print(input-prevInput);
      Serial.print(" ");
      Serial.print(output);
      Serial.print(" ");
      Serial.println(reflowStatusName[reflowState]);
    }
    else
    {
      // Turn off red LED
      digitalWrite(LED_PIN, LOW);

      Serial.print(reflowStatusName[reflowState]);
      Serial.print(" temp=");reflowStatusName[reflowState];
      Serial.println(input);
    }

    // If currently in error state
    if (reflowState == REFLOW_STATE_ERROR)
    {
      // No thermocouple wire connected
      Serial.println("TC Error!");
    }
  }

  // Reflow oven controller state machine
  switch (reflowState)
  {
  case REFLOW_STATE_IDLE:
// If oven temperature is still above room temperature
    if (input >= TEMPERATURE_ROOM)
    {
      reflowState = REFLOW_STATE_TOO_HOT;
    }
    else if (startRequested)
    {
      // Send header for CSV file
      Serial.println("Time Setpoint Input Delta Output State");
      // Intialize seconds timer for serial debug information
      timerSeconds = 0;
      // Initialize PID control window starting time
      windowStartTime = millis();
      // Ramp up to minimum soaking temperature
      setpoint = TEMPERATURE_SOAK_MIN;
      // Tell the PID to range between 0 and the full window size
      reflowOvenPID.SetOutputLimits(0, SSR_PWM_PERIOD);
      reflowOvenPID.SetSampleTime(PID_SAMPLE_TIME);
      reflowOvenPID.SetTunings(PID_KP_PREHEAT, PID_KI_PREHEAT, PID_KD_PREHEAT);
      // Turn the PID on
      reflowOvenPID.SetMode(AUTOMATIC);
      // Proceed to preheat stage
      reflowState = REFLOW_STATE_PREHEAT;
    }
    startRequested = false;
    break;

  case REFLOW_STATE_PREHEAT:
    reflowStatus = REFLOW_STATUS_ON;
    // If minimum soak temperature is achieve       
    if (input >= TEMPERATURE_SOAK_MIN)
    {
      // Chop soaking period into smaller sub-period
      timerSoak = millis() + SOAK_MICRO_PERIOD;
      // Set less agressive PID parameters for soaking ramp
      reflowOvenPID.SetTunings(PID_KP_SOAK, PID_KI_SOAK, PID_KD_SOAK);
      // Ramp up to first section of soaking temperature
      setpoint = TEMPERATURE_SOAK_MIN + SOAK_TEMPERATURE_STEP;   
      // Proceed to soaking state
      reflowState = REFLOW_STATE_SOAK; 
    }
    break;

  case REFLOW_STATE_SOAK:     
    // If micro soak temperature is achieved       
    if (millis() > timerSoak)
    {
      timerSoak = millis() + SOAK_MICRO_PERIOD;
      // Increment micro setpoint
      setpoint += SOAK_TEMPERATURE_STEP;
      if (setpoint > TEMPERATURE_SOAK_MAX)
      {
        // Set agressive PID parameters for reflow ramp
        reflowOvenPID.SetTunings(PID_KP_REFLOW, PID_KI_REFLOW, PID_KD_REFLOW);
        // Ramp up to first section of soaking temperature
        setpoint = TEMPERATURE_REFLOW_MAX;   
        // Proceed to reflowing state
        reflowState = REFLOW_STATE_REFLOW; 
      }
    }
    break; 

  case REFLOW_STATE_REFLOW:
    // We need to avoid hovering at peak temperature for too long
    // Crude method that works like a charm and safe for the components
    if (input >= (TEMPERATURE_REFLOW_MAX - 5))
    {
      // Set PID parameters for cooling ramp
      reflowOvenPID.SetTunings(PID_KP_REFLOW, PID_KI_REFLOW, PID_KD_REFLOW);
      // Ramp down to minimum cooling temperature
      setpoint = TEMPERATURE_COOL_MIN;   
      // Proceed to cooling state
      reflowState = REFLOW_STATE_COOL; 
    }
    break;   

  case REFLOW_STATE_COOL:
    // If minimum cool temperature is achieve       
    if (input <= TEMPERATURE_COOL_MIN)
    {
      // Turn off reflow process
      reflowStatus = REFLOW_STATUS_OFF;                
      // Proceed to reflow Completion state
      reflowState = REFLOW_STATE_IDLE; 
    }         
    break;
    
  case REFLOW_STATE_TOO_HOT:
    // If oven temperature drops below room temperature
    if (input < TEMPERATURE_ROOM)
    {
      // Ready to reflow
      reflowState = REFLOW_STATE_IDLE;
    }
    break;
		
  case REFLOW_STATE_ERROR:
    // If thermocouple problem is still present
		#ifdef	USE_MAX6675
			if (isnan(input))
		#else
			if((input == FAULT_OPEN) || (input == FAULT_SHORT_GND) || 
				 (input == FAULT_SHORT_VCC))
    #endif
    {
      // Wait until thermocouple wire is connected
      reflowState = REFLOW_STATE_ERROR; 
    }
    else
    {
      // Clear to perform reflow process
      reflowState = REFLOW_STATE_IDLE; 
    }
    break;    
  }    

  // If switch 1 is pressed
  while (Serial.available())
  {
    if (Serial.read() == 's')
    {
      if (reflowStatus == REFLOW_STATUS_ON) {
        // Button press is for cancelling
        // Turn off reflow process
        reflowStatus = REFLOW_STATUS_OFF;
        // Reinitialize state machine
        reflowState = REFLOW_STATE_IDLE;
      } else {
        startRequested = true;
      }
    }
  }

  // PID computation and SSR control
  if (reflowStatus == REFLOW_STATUS_ON)
  {
    now = millis();

    reflowOvenPID.Compute();

    if((now - windowStartTime) > SSR_PWM_PERIOD)
    { 
      // Time to shift the Relay Window
      windowStartTime += SSR_PWM_PERIOD;
    }
    if(output > (now - windowStartTime)) digitalWrite(SSR_PIN, HIGH);
    else digitalWrite(SSR_PIN, LOW);   
  }
  // Reflow oven process is off, ensure oven is off
  else 
  {
    digitalWrite(SSR_PIN, LOW);
  }
}
