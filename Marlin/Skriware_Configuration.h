#ifndef SK_CONFIG
#define SK_CONFIG

//Debug settings:
//#define SKRIWARE_DEBUG
//#define DEBUG_E_FADE
//#define LAYER_DEBUG

//MAIN SKRIWARE SETTINGS:

//#define MOVING_EXTRUDER
//#define OPTICAL_SENSOR
//#define EXT_CHECKSTATION

/**************************** PRINTER SETUP ********************************************/
#define MOTHERBOARD BOARD_MKS_GEN_13

#define BAUDRATE 115200

#define SERIAL_PORT 0

#define HOST_KEEPALIVE_FEATURE        // Disable this if your host doesn't like keepalive messages
#define DEFAULT_KEEPALIVE_INTERVAL 2  // Number of seconds between "busy" messages. Set with M113.

#define HOTEND_OFFSET_X {0.0, -30.5} // (in mm) for each extruder, offset of the hotend on the X axis
#define HOTEND_OFFSET_Y {0.0, 0.0}  // (in mm) for each extruder, offset of the hotend on the Y axis

#define EXTRUDERS 2

#define COREXY

#define USE_XMIN_PLUG
#define USE_YMIN_PLUG
#define USE_ZMIN_PLUG

#define X_MIN_ENDSTOP_INVERTING true // set to true to invert the logic of the endstop.
#define Y_MIN_ENDSTOP_INVERTING true // set to true to invert the logic of the endstop.
#define Z_MIN_ENDSTOP_INVERTING true // set to true to invert the logic of the endstop.
#define X_MAX_ENDSTOP_INVERTING false // set to true to invert the logic of the endstop.
#define Y_MAX_ENDSTOP_INVERTING false // set to true to invert the logic of the endstop.
#define Z_MAX_ENDSTOP_INVERTING false // set to true to invert the logic of the endstop.
#define Z_MIN_PROBE_ENDSTOP_INVERTING true // set to true to invert the logic of the probe.


/************************** Z PROBE **************************************************/

#define Z_MIN_PROBE_USES_Z_MIN_ENDSTOP_PIN
#define FIX_MOUNTED_PROBE

#define X_PROBE_OFFSET_FROM_EXTRUDER 20  // X offset: -left  +right  [of the nozzle]
#define Y_PROBE_OFFSET_FROM_EXTRUDER 30  // Y offset: -front +behind [the nozzle]
#define Z_PROBE_OFFSET_FROM_EXTRUDER 0   // Z offset: -below +above  [the nozzle]

#define MIN_PROBE_EDGE 5
// X and Y axis travel speed (mm/m) between probes
#define XY_PROBE_SPEED 4800

// Speed for the first approach when double-probing (with PROBE_DOUBLE_TOUCH)
#define Z_PROBE_SPEED_FAST HOMING_FEEDRATE_Z

// Speed for the "accurate" probe of each point
#define Z_PROBE_SPEED_SLOW (Z_PROBE_SPEED_FAST / 2)

// Use double touch for probing
#define MULTIPLE_PROBING 2

#define Z_CLEARANCE_DEPLOY_PROBE   0 // Z Clearance for Deploy/Stow
#define Z_CLEARANCE_BETWEEN_PROBES  5 // Z Clearance between probe points

// For M851 give a range for adjusting the Z probe offset
#define Z_PROBE_OFFSET_RANGE_MIN -20
#define Z_PROBE_OFFSET_RANGE_MAX 20

// Enable the M48 repeatability test to test probe accuracy
#define Z_MIN_PROBE_REPEATABILITY_TEST


#define X_ENABLE_ON 0
#define Y_ENABLE_ON 0
#define Z_ENABLE_ON 0
#define E_ENABLE_ON 0 // For all extruders

#define DISABLE_X false
#define DISABLE_Y false
#define DISABLE_Z false
#define DISABLE_E false // For all extruders

#define DISABLE_INACTIVE_EXTRUDER true // Keep only the active extruder enabled.

#define INVERT_X_DIR true
#define INVERT_Y_DIR true
#define INVERT_Z_DIR true

#define INVERT_E0_DIR false
#define INVERT_E1_DIR false
#define INVERT_E2_DIR false
#define INVERT_E3_DIR false
#define INVERT_E4_DIR false


/**************************** AXIS **************************************************/

#define X_HOME_DIR -1
#define Y_HOME_DIR -1
#define Z_HOME_DIR -1

#define X_MIN_POS 0
#define Y_MIN_POS 0
#define Z_MIN_POS -3

#define X_BED_SIZE 210
#define Y_BED_SIZE 275

#define X_MAX_POS 210
#define Y_MAX_POS 275
#define Z_MAX_POS 210

#define MIN_SOFTWARE_ENDSTOPS

#define MAX_SOFTWARE_ENDSTOPS

#define QUICK_HOME 


/***************************************************************************************/

/**************************** HEATING **************************************************/
#define TEMP_SENSOR_0 1
#define TEMP_SENSOR_1 1
#define TEMP_SENSOR_2 0
#define TEMP_SENSOR_3 0
#define TEMP_SENSOR_4 0
#define TEMP_SENSOR_BED 1

#define DUMMY_THERMISTOR_998_VALUE 25
#define DUMMY_THERMISTOR_999_VALUE 100

#define MAX_REDUNDANT_TEMP_SENSOR_DIFF 10

// Extruder temperature must be close to target for this long before M109 returns success
#define TEMP_RESIDENCY_TIME 3  // (seconds)
#define TEMP_HYSTERESIS 3       // (degC) range of +/- temperatures considered "close" to the target one
#define TEMP_WINDOW     1       // (degC) Window around target to start the residency timer x degC early.

// Bed temperature must be close to target for this long before M190 returns success
#define TEMP_BED_RESIDENCY_TIME 3  // (seconds)
#define TEMP_BED_HYSTERESIS 3       // (degC) range of +/- temperatures considered "close" to the target one
#define TEMP_BED_WINDOW     1  

#define HEATER_0_MAXTEMP 275
#define HEATER_1_MAXTEMP 275
#define HEATER_2_MAXTEMP 275
#define HEATER_3_MAXTEMP 275
#define HEATER_4_MAXTEMP 275
#define BED_MAXTEMP 20

#define HEATER_0_MINTEMP 5
#define HEATER_1_MINTEMP 5
#define HEATER_2_MINTEMP 5
#define HEATER_3_MINTEMP 5
#define HEATER_4_MINTEMP 5
#define BED_MINTEMP 5

#define PREVENT_COLD_EXTRUSION
#define EXTRUDE_MINTEMP 170

#define PREVENT_LENGTHY_EXTRUDE
#define EXTRUDE_MAXLENGTH 200

#define THERMAL_PROTECTION_HOTENDS // Enable thermal protection for all extruders
#define THERMAL_PROTECTION_BED     // Enable thermal protection for the heated bed

#define PREHEAT_1_TEMP_HOTEND 180
#define PREHEAT_1_TEMP_BED     70
#define PREHEAT_1_FAN_SPEED     0 // Value from 0 to 255

#define PREHEAT_2_TEMP_HOTEND 240
#define PREHEAT_2_TEMP_BED    110
#define PREHEAT_2_FAN_SPEED     0 // Value from 0 to 255


/***************************************************************************************/


/**************************** PID ******************************************************/
#define PIDTEMP
#define BANG_MAX 255 // limits current to nozzle while in bang-bang mode; 255=full current
#define PID_MAX BANG_MAX // limits current to nozzle while PID is active (see PID_FUNCTIONAL_RANGE below); 255=full current
#define PID_PARAMS_PER_HOTEND // Uses separate PID parameters for each extruder (useful for mismatched extruders)
                                  // Set/get with gcode: M301 E[extruder number, 0-2]
#define PID_FUNCTIONAL_RANGE 10 // If the temperature difference between the target temperature and the actual temperature
                                  // is more than PID_FUNCTIONAL_RANGE then the PID will be shut off and the heater will be set to min/max.

#define  PID_K1 0.95 //smoothing factor within the PID
#define  DEFAULT_Kp 17.23
#define  DEFAULT_Ki 1.17
#define  DEFAULT_Kd 63.17

#define PIDTEMPBED
#define MAX_BED_POWER 255

#if ENABLED(PIDTEMPBED)
#define  DEFAULT_bedKp 158.23
#define  DEFAULT_bedKi 23.27
#define  DEFAULT_bedKd 268.98
#endif
/***************************************************************************************/

/**************************** BED LEVELING *********************************************/
#define AUTO_BED_LEVELING_BILINEAR
#define DEBUG_LEVELING_FEATURE

#define ENABLE_LEVELING_FADE_HEIGHT

  #define GRID_MAX_POINTS_X 5
  #define GRID_MAX_POINTS_Y 5//GRID_MAX_POINTS_X

  // Set the boundaries for probing (where the probe can reach).
  #define LEFT_PROBE_BED_POSITION 20
  #define RIGHT_PROBE_BED_POSITION 180
  #define FRONT_PROBE_BED_POSITION 32
  #define BACK_PROBE_BED_POSITION 255

  // The Z probe minimum outer margin (to validate G29 parameters).
  #define MIN_PROBE_EDGE 0
  #define EXTRAPOLATE_BEYOND_GRID
  #define ABL_BILINEAR_SUBDIVISION
  #if ENABLED(ABL_BILINEAR_SUBDIVISION)
   	#define BILINEAR_SUBDIVISIONS 3
  #endif

#define MANUAL_X_HOME_POS 0
#define MANUAL_Y_HOME_POS 0
#define MANUAL_Z_HOME_POS 0
/***************************************************************************************/

/**************************** SPEED & ACCELERATIONS ************************************/

#define DEFAULT_AXIS_STEPS_PER_UNIT   { 80, 80, 400, 204} 
#define DEFAULT_MAX_FEEDRATE          { 1000, 1000, 50, 100 }
#define DEFAULT_MAX_ACCELERATION      { 1500, 1500, 50, 1500}

#define DEFAULT_ACCELERATION          3000    // X, Y, Z and E acceleration for printing moves
#define DEFAULT_RETRACT_ACCELERATION  3000    // E acceleration for retracts
#define DEFAULT_TRAVEL_ACCELERATION   1500    // X, Y, Z acceleration for travel (non printing) moves

#define DEFAULT_XJERK                 5.0
#define DEFAULT_YJERK                 5.0
#define DEFAULT_ZJERK                 0.4
#define DEFAULT_EJERK                 5.0

#define HOMING_FEEDRATE_XY (40*60)
#define HOMING_FEEDRATE_Z  200

/***************************************************************************************/

/*********************************EXTRUDER CHECKSTATIOB ********************************/
#ifdef EXT_CHECKSTATION
  #define MOVING_EXTRUDER
	#define EXTRUDE_MINTEMP -30 
	//#define USE_ZMAX_PLUG 
	#define DEFAULT_AXIS_STEPS_PER_UNIT   {80, 800,2560,204}     
	#define DEFAULT_MAX_FEEDRATE          {1000, 1000, 500, 100}
  #define DEFAULT_MAX_ACCELERATION      {100,100, 10, 1500} 
	#define Z_MIN_POS -200
  #define Z_MIN_ENDSTOP_INVERTING false   
  #define Z_MIN_PROBE_ENDSTOP_INVERTING false
	#define INVERT_X_DIR true
  #define Z_HOMING_HEIGHT 0
  #undef COREXY
#endif


/****************************************************************************************/

/*********************************EEPROM ************************************************/
#define EEPROM_SETTINGS
#define EEPROM_CHITCHAT

#define PRINTJOB_TIMER_AUTOSTART

#define PRINTCOUNTER

#define POWER_SUPPLY 0

#define STRING_CONFIG_H_AUTHOR "(none, default config)" // Who made the changes.
#define SHOW_BOOTSCREEN
#define STRING_SPLASH_LINE1 SHORT_BUILD_VERSION // will be shown during bootup in line 1
#define STRING_SPLASH_LINE2 WEBSITE_URL         // will be shown during bootup in line 2


/****************************************************************************************/

#define SOFT_PWM_SCALE 0
#define DEFAULT_NOMINAL_FILAMENT_DIA 3.00

/**************************** SKRIWARE ONLY FUNCTIONS *********************************/

#if ENABLED(AUTO_BED_LEVELING_BILINEAR)
#define E_FADE                                
#define E_FADE_MAX_LAYER_HIGH 0.25
#endif

#define START_GCODE_EXTRUSION_CORRECTION

//Moving extruder variables:
#ifdef MOVING_EXTRUDER
  #define HAS_SERVO
  #define NUM_SERVOS 1
  #define SERVO_DELAY {300}
  #define SERVO_POS_DOWN 80
  #define SERVO_POS_UP   33
  #define MOTOR_UP_TIME 2000
  #define EXT_CHANGE_TIME_OFFSET 500
  #define DEF_EXTRDER_TYPE 0
  #define Y_MIN_POS -10
#endif



/***************************************************************************************/

/**************************** FILAMENT SENSORS *****************************************/
#ifdef OPTICAL_SENSOR
	#define OPTICAL_SENSOR_INT_TIME 500
	#define OPTICAL_SENSOR_MEASUREMENT_TIME 100
	#define OPTICAL_SENSOR_N_TO_MEAN 20
  #define OPTICAL_SENSOR_ERROR_LEVEL 8 
  #define OPTICAL_SENSOR_NOISE_OFFSET 30
#endif

#define FILAMENT_JAM_SENSOR     //ukikoza
#if ENABLED(FILAMENT_JAM_SENSOR)
#define FILAMENT_JAM_ALARM 500
#define FILAMENT_JAM_ERROR 700
#define FILAMENT_JAM_SENSOR_PIN_E0 15
#define FILAMENT_JAM_SENSOR_PIN_E1 2
#define FILAMET_JAM_SENSOR_TURN_ON_RETRACT_BUFFOR 800
//#define FILAMENT_JAM_SENSOR_DEBUG
#endif

#define SKRIWARE_FILAMENT_RUNOUT_SENSOR
#ifdef SKRIWARE_FILAMENT_RUNOUT_SENSOR
  #define SKRIWARE_FILAMENT_RUNOUT_INVERTING
  #define SKRIWARE_FILAMENT_RUNOUT_SENSOR_PIN_E0 15
  #define SKRIWARE_FILAMENT_RUNOUT_SENSOR_PIN_E1 2
  #define BINARY_SENSOR_DEBOUNCE_TIME 500
#endif

/***************************************************************************************/

#endif