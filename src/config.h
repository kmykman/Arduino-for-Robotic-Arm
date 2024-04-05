#ifndef CONFIG_H_
#define CONFIG_H_

//ROBOT ARM LENGTH
//#define SHANK_LENGTH 140.0
#define LOW_SHANK_LENGTH 180.0
#define HIGH_SHANK_LENGTH 180.0

#define END_EFFECTOR_OFFSET 35.0 // LENGTH FROM UPPER SHANK BEARING TO MIDPOINT OF END EFFECTOR IN MM

//INITIAL INTERPOLATION SETTINGS
//  INITIAL_XYZ FORMS VERTICAL LOWER ARM & HORIZONTAL UPPER ARM IN 90 DEGREES
#define INITIAL_X 0.0 // CARTESIAN COORDINATE X  
#define INITIAL_Y (HIGH_SHANK_LENGTH+END_EFFECTOR_OFFSET) // CARTESIAN COORDINATE Y
#define INITIAL_Z LOW_SHANK_LENGTH // CARTESIAN COORDINATE Z

#define INITIAL_E0 0.0 // RAIL STEPPER ENDSTOP POSITION 

//  CALIBRATE HOME STEPS TO REACH DESIRED INITIAL_XYZ POSITIONS
#define X_HOME_STEPS 300 //1020 //765 //860 // STEPS FROM X_ENDSTOP TO INITIAL_XYZ FOR UPPER ARM
#define Y_HOME_STEPS 1640 //1900 //1940 // STEPS FROM Y_ENDSTOP TO INITIAL_XYZ FOR LOWER ARM
#define Z_HOME_STEPS 1730// 1820 //3640 // STEPS FROM Z_ENDSTOP TO INITIAL_XYZ FOR ROTATION CENTER

//HOMING SETTINGS:
#define HOME_X_STEPPER true // "true" IF ENDSTOP IS INSTALLED
#define HOME_Y_STEPPER true // "true" IF ENDSTOP IS INSTALLED
#define HOME_Z_STEPPER true // "true" IF ENDSTOP IS INSTALLED

#define HOME_ON_BOOT false // "true" IF HOMING REQUIRED AFTER POWER ON
#define HOME_DWELL 800 // INCREASE TO SLOW DOWN HOMING SPEED

//STEPPER SETTINGS:

#define INVERSE_X_STEPPER false // CHANGE IF STEPPER MOVES OTHER WAY
#define INVERSE_Y_STEPPER false // CHANGE IF STEPPER MOVES OTHER WAY
#define INVERSE_Z_STEPPER true // CHANGE IF STEPPER MOVES OTHER WAY

//ENDSTOP SETTINGS:
#define X_MIN_INPUT 0 // OUTPUT VALUE WHEN SWITCH ACTIVATED - NO: 0, NC: 1
#define Y_MIN_INPUT 0 // OUTPUT VALUE WHEN SWITCH ACTIVATED - NO: 0, NC: 1
#define Z_MIN_INPUT 0 // OUTPUT VALUE WHEN SWITCH ACTIVATED - NO: 0, NC: 1

//MOVE LIMIT PARAMETERS
#define Z_MIN -80.0 //MINIMUM Z HEIGHT OF TOOLHEAD TOUCHING GROUND
#define Z_MAX (LOW_SHANK_LENGTH+30.0) //SHANK_LENGTH ADDING ARBITUARY NUMBER FOR Z_MAX
#define SHANKS_MIN_ANGLE_COS 0.791436948 
#define SHANKS_MAX_ANGLE_COS -0.774944489 
#define R_MIN (sqrt((sq(LOW_SHANK_LENGTH) + sq(HIGH_SHANK_LENGTH)) - (2*LOW_SHANK_LENGTH*HIGH_SHANK_LENGTH*SHANKS_MIN_ANGLE_COS) ))
#define R_MAX (sqrt((sq(LOW_SHANK_LENGTH) + sq(HIGH_SHANK_LENGTH)) - (2*LOW_SHANK_LENGTH*HIGH_SHANK_LENGTH*SHANKS_MAX_ANGLE_COS) ))

#endif
