/*
**==============================================================================
** CONFIG.C:             -- by Dr. ZhuoQing, 2012-2-6
**
**==============================================================================
*/

//------------------------------------------------------------------------------
#define CONFIG_GLOBALS        1                       // Define the global variables
#include "CONFIG.H"
//==============================================================================

//------------------------------------------------------------------------------
//==============================================================================
//						CONFIGRATION 
//------------------------------------------------------------------------------
void ArgumentInit(void) 
{
	g_nGravityOffset 		        = GRAVITY_OFFSET_DEFAULT;
	g_nGyroscopeOffset		= GYROSCOPE_OFFSET_DEFAULT;
	g_nGravityMax			= GRAVITY_MAX_DEFAULT;
	g_nGravityMin			= GRAVITY_MIN_DEFAULT;
	g_fAngleControlP		        = ANGLE_CONTROL_P_DEFAULT;
	g_fAngleControlD		        = ANGLE_CONTROL_D_DEFAULT;
	g_fSpeedControlP		        = SPEED_CONTROL_P_DEFAULT;
	g_fSpeedControlI		        = SPEED_CONTROL_I_DEFAULT;
	g_fDirectionControlP              = DIRECTION_CONTROL_P_DEFAULT;
	g_nDirLeftOffset		        = DIR_LEFT_OFFSET_DEFAULT;
	g_nDirRightOffset		        = DIR_RIGHT_OFFSET_DEFAULT;
	g_fDeadVoltage			= DEAD_VOLTAGE_DEFAULT;
	g_fGyroscopeAngleRatio            = GYROSCOPE_ANGLE_RATIO_DEFAULT;
	g_fGravityTimeConstant            = GRAVITY_TIME_CONSTANT_DEFAULT;
	g_fCarSpeedSet		 	= CAR_SPEED_SET_DEFAULT;
	g_nPad					= 0;
	g_nInitFlag			= INIT_FLAG_DEFAULT;
        g_fStarttimeCount                  = start_time_count;
}

/*
void ArgumentLoad(void) {
	pmemReadDim(ARGUMENT_ADDRESS, sizeof(g_Argument) / 2, (unsigned int *)&g_Argument);
}
			
void ArgumentSave(void) {
	unsigned int * pArgument;
	int nSize, i;
	
	nSize = sizeof(g_Argument);
	pArgument = (unsigned int *)&g_Argument;
	for(i = 0; i < (nSize + 1) / 2; i ++) {
		IFsh1_SetWordFlash(i + ARGUMENT_ADDRESS, *(pArgument + i));
	}
}
*/
//==============================================================================
//                END OF THE FILE : CONFIG.C
//------------------------------------------------------------------------------
