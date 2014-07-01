/*******************************************************************************
* FILE NAME: user_routines.c <VEX VERSION>
*
* DESCRIPTION:
*  This file contains the joystick and autonomous robot control for the 4-wheel
*  drive, spoke-wheel design. Heavily modified by Bereket Abraham.  
*
* USAGE:
*  You can either modify this file to fit your needs, or remove it from your 
*  project and replace it with a modified copy. 
*
*******************************************************************************/

#include "ifi_aliases.h"
#include "ifi_default.h"
#include "ifi_utilities.h"
#include "user_routines.h"
#include "printf_lib.h"

#define CODE_VERSION            10

#define BUTTON_REV_THRESH       100
#define BUTTON_FWD_THRESH       154
#define NEUTRAL_VALUE           127
#define open     200
#define closed     0
#define period     250

float Left_Side = 0.0;  // -1.0 to 1.0
float Right_Side = 0.0;  // -1.0 to 1.0 
float divisor = 4.0;
unsigned int auto_mode = 0;
unsigned int slow_mode = 1;
unsigned int counter = 0;
unsigned int drive_state = 0;
unsigned int fix_turn = 1;
unsigned int btn_count = 0;
unsigned int btn_count2 = 0;
unsigned int arm_count = 0;
unsigned int light_count = 0;
int arm_pwm = 127;
int hand_pwm = 0;


// PURPOSE:       Limits the mixed value for one joystick drive.
unsigned char Limit_Mix (int intermediate_value)
{
  static int limited_value;
  
  if (intermediate_value < 2000)
  {
    limited_value = 2000;
  }
  else if (intermediate_value > 2254)
  {
    limited_value = 2254;
  }
  else
  {
    limited_value = intermediate_value;
  }
  return (unsigned char) (limited_value - 2000);
}


/*******************************************************************************
* FUNCTION NAME: Setup_Who_Controls_Pwms
* PURPOSE:       Each parameter specifies what processor will control the pwm.  
*                 
* CALLED FROM:   User_Initialization
*     Argument             Type    IO   Description
*     --------             ----    --   -----------
*     pwmSpec1              int     I   USER/MASTER (defined in ifi_aliases.h)
*     pwmSpec2              int     I   USER/MASTER
*     pwmSpec3              int     I   USER/MASTER
*     pwmSpec4              int     I   USER/MASTER
*     pwmSpec5              int     I   USER/MASTER
*     pwmSpec6              int     I   USER/MASTER
*     pwmSpec7              int     I   USER/MASTER
*     pwmSpec8              int     I   USER/MASTER
* RETURNS:       void
*******************************************************************************/
static void Setup_Who_Controls_Pwms(int pwmSpec1,int pwmSpec2,int pwmSpec3,int pwmSpec4,
                                    int pwmSpec5,int pwmSpec6,int pwmSpec7,int pwmSpec8)
{
  txdata.pwm_mask = 0xFF;         /* Default to master controlling all PWMs. */
  if (pwmSpec1 == USER)           /* If User controls PWM1 then clear bit0. */
    txdata.pwm_mask &= 0xFE;      /* same as txdata.pwm_mask = txdata.pwm_mask & 0xFE; */
  if (pwmSpec2 == USER)           /* If User controls PWM2 then clear bit1. */
    txdata.pwm_mask &= 0xFD;
  if (pwmSpec3 == USER)           /* If User controls PWM3 then clear bit2. */
    txdata.pwm_mask &= 0xFB;
  if (pwmSpec4 == USER)           /* If User controls PWM4 then clear bit3. */
    txdata.pwm_mask &= 0xF7;
  if (pwmSpec5 == USER)           /* If User controls PWM5 then clear bit4. */
    txdata.pwm_mask &= 0xEF;
  if (pwmSpec6 == USER)           /* If User controls PWM6 then clear bit5. */
    txdata.pwm_mask &= 0xDF;
  if (pwmSpec7 == USER)           /* If User controls PWM7 then clear bit6. */
    txdata.pwm_mask &= 0xBF;
  if (pwmSpec8 == USER)           /* If User controls PWM8 then clear bit7. */
    txdata.pwm_mask &= 0x7F;
}


/*******************************************************************************
* FUNCTION NAME: User_Initialization
* PURPOSE:       This routine is called first (and only once) in the Main function.  
*                You may modify and add to this function.
*                The primary purpose is to set up the DIGITAL IN/OUT - ANALOG IN
*                pins as analog inputs, digital inputs, and digital outputs.
* CALLED FROM:   main.c
* ARGUMENTS:     none
* RETURNS:       void
*******************************************************************************/
void User_Initialization (void)
{
/* FIRST: Set up the pins you want to use as analog INPUTs. */
  IO1 = IO2 = IO3 = INPUT;        /* Used for analog inputs. */

/* SECOND: Configure the number of analog channels. */
  Set_Number_of_Analog_Channels(THREE_ANALOG);     /* See ifi_aliases.h */

/* THIRD: Set up any extra digital inputs. */
  /* The six INTERRUPTS are already digital inputs. */
  /* If you need more then set them up here. */
  /* IOxx = IOyy = INPUT; */
  IO3 = IO4 = IO5 = IO6 = IO7 = IO8 = INPUT;        
  IO9 = IO10 = IO11 = IO12 = IO13 = IO15 = INPUT;    

/* FOURTH: Set up the pins you want to use as digital OUTPUTs. */
  IO14 = IO16 = OUTPUT;  

/* FIFTH: Initialize the values on the digital outputs. */
  rc_dig_out14 = rc_dig_out16 = 0;

/* SIXTH: Set your initial PWM values.  Neutral is 127. */
  pwm01 = pwm02 = pwm03 = pwm04 = pwm05 = pwm06 = pwm07 = pwm08 = 127;

/* SEVENTH: Choose which processor will control which PWM outputs. */
  Setup_Who_Controls_Pwms(MASTER,MASTER,MASTER,MASTER,MASTER,MASTER,MASTER,MASTER);

/* EIGHTH: Set your PWM output type.  Only applies if USER controls PWM 1, 2, 3, or 4. */
  /*   Choose from these parameters for PWM 1-4 respectively:                          */
  /*     IFI_PWM  - Standard IFI PWM output generated with Generate_Pwms(...)          */
  /*     USER_CCP - User can use PWM pin as digital I/O or CCP pin.                    */
  Setup_PWM_Output_Type(IFI_PWM,IFI_PWM,IFI_PWM,IFI_PWM);
  
  /* 
     Example: The following would generate a 40KHz PWM with a 50% duty cycle
              on the CCP2 pin (PWM OUT 1):

      Setup_Who_Controls_Pwms(USER,USER,MASTER,MASTER,MASTER,MASTER,MASTER,MASTER);
      CCP2CON = 0x3C;
      PR2 = 0xF9;
      CCPR2L = 0x7F;
      T2CON = 0;
      T2CONbits.TMR2ON = 1;
      Setup_PWM_Output_Type(USER_CCP,IFI_PWM,IFI_PWM,IFI_PWM);
  */

/* Add any other user initialization code here. */

  Initialize_Serial_Comms();     
 
  Putdata(&txdata);             /* DO NOT CHANGE! */
  User_Proc_Is_Ready();         /* DO NOT CHANGE! - last line of User_Initialization */


#ifdef _SIMULATOR
    statusflag.NEW_SPI_DATA = 1;
#else
  /* This code receives the 1st packet from master to obtain the version # */
  while (!statusflag.NEW_SPI_DATA);  /* Wait for 1st packet from master */
  Getdata(&rxdata);   
  printf("VEX - Master v%d, User v%d\n",(int)rxdata.master_version,(int)CODE_VERSION);   
     
#endif
}


// convert the [-1.0, 1.0] range to t he real operational range of the left back motor
unsigned char Set_LB_Motor(float motor_val)
{
  // operational range
  int for_bottom = 135;
  int for_top = 167;
  int rev_bottom = 121;
  int rev_top = 99;
  int pwm = 0;
  
  motor_val = motor_val / divisor;
  
  if (motor_val > 0.001) {   // room for float pt errors
    motor_val = ((float)(for_top - for_bottom)) * motor_val;
    pwm = (int)motor_val + for_bottom;
  }
  else if (motor_val < -0.001) {
    motor_val = ((float)(rev_bottom - rev_top)) * motor_val;
    pwm = (int)motor_val + rev_bottom;
  }
  else {
    pwm = 127;
  }

  if (pwm > 255) { pwm = 255; }
  if (pwm < 0) { pwm = 0; }
 
  return (unsigned char)pwm;
}


// convert the [-1.0, 1.0] range to the real operational range of the right back motor
unsigned char Set_RB_Motor(float motor_val)
{
  // operational range
  int for_bottom = 145;
  int for_top = 211;
  int rev_bottom = 114;
  int rev_top = 45;
  int pwm = 0;
  
  motor_val = motor_val / divisor;
  
  if (motor_val > 0.001) {   // room for float pt errors
    motor_val = ((float)(for_top - for_bottom)) * motor_val;
    pwm = (int)motor_val + for_bottom;
  }
  else if (motor_val < -0.001) {
    motor_val = ((float)(rev_bottom - rev_top)) * motor_val;
    pwm = (int)motor_val + rev_bottom;
  }
  else {
    pwm = 127;
  }


  if (pwm > 255) { pwm = 255; }
  if (pwm < 0) { pwm = 0; }
 
  return (unsigned char)pwm;
}


// convert the [-1.0, 1.0] range to the real operational range of the left front motor
unsigned char Set_LF_Motor(float motor_val)
{
  // operational range
  int for_bottom = 141;
  int for_top = 220;
  int rev_bottom = 109;
  int rev_top = 45;
  int pwm = 0;
  
  motor_val = motor_val / divisor;
  
  if (motor_val > 0.001) {   // room for float pt errors
    motor_val = ((float)(for_top - for_bottom)) * motor_val;
 pwm = (int)motor_val + for_bottom;
  }
  else if (motor_val < -0.001) {
    motor_val = ((float)(rev_bottom - rev_top)) * motor_val;
 pwm = (int)motor_val + rev_bottom;
  }
  else {
    pwm = 127;
  }

  if (pwm > 255) { pwm = 255; }
  if (pwm < 0) { pwm = 0; }
 
  return (unsigned char)pwm;
}


 
// convert the [-1.0, 1.0] range to the real operational range of the right front motor
unsigned char Set_RF_Motor(float motor_val)
{
  // operational range
  int for_bottom = 141;
  int for_top = 215;
  int rev_bottom = 116;
  int rev_top = 45;
  int pwm = 0;
  
  motor_val = motor_val / divisor;
  
  if (motor_val > 0.001) {   // room for float pt errors
    motor_val = ((float)(for_top - for_bottom)) * motor_val;
 pwm = (int)motor_val + for_bottom;
  }
  else if (motor_val < -0.001) {
    motor_val = ((float)(rev_bottom - rev_top)) * motor_val;
 pwm = (int)motor_val + rev_bottom;
  }
  else {
    pwm = 127;
  }

  if (pwm > 255) { pwm = 255; }
  if (pwm < 0) { pwm = 0; }
 
  return (unsigned char)pwm;
}


 
// convert the operational range of the left light sensor to a [0,1.0] range
float Set_L_Light_Sensor(int left_eye)
{
  // operational range
  int maxL = 700;   // ~ dark
  int minL = 80;   // ~ 1ft away
  float val = 0;
  
  val = ((float)(left_eye - minL)) / ((float)(maxL - minL));
  return val;
}

// convert the operational range of the right light sensor to a [0,1.0] range
float Set_R_Light_Sensor(int right_eye)
{
  // operational range
  int maxR = 1050;   // ~ dark
  int minR = 300;   // ~ 1ft away
  float val = 0;
  
  val = ((float)(right_eye - minR)) / ((float)(maxR - minR));
  return val;
}

 

// convert the operational range of the left light sensor to a [0,1.0] range
float Set_L_Prox(int left_prox)
{
  // operational range
  int maxL = 500;   // ~ 2 inches away
  int minL = 5;   // ~ infinite away
  float val = 0;
  
  val = ((float)(left_prox - minL)) / ((float)(maxL - minL));
  return val;
}

// convert the operational range of the right light sensor to a [0,1.0] range
float Set_R_Prox(int right_prox)
{
  // operational range
  int maxR = 500;   // ~ 2 inches away
  int minR = 5;   // ~ infinite away
  float val = 0;
  
  val = ((float)(right_prox - minR)) / ((float)(maxR - minR));
  return val;
}

void Process_Driving_State(int drive_state)
{
  switch (drive_state) {
  case 0:     // stop
    Right_Side = 0.0;
    Left_Side = 0.0;
    break;
  case 1:     // straight
    Right_Side = 0.6;
    Left_Side = 0.6;
    break;
  case 2:     // reverse
    Right_Side = -0.6;
    Left_Side = -0.6;
    break;
  case 3:     // turn right
    Right_Side = 0.0;
    Left_Side = 0.8;
    break;
  case 4:     // turn left
    Right_Side = 0.8;
    Left_Side = 0.0;
    break;
  case 5:     // turn right rev
    Right_Side = 0.0;
    Left_Side = -0.9;
    break;
  case 6:     // turn left rev
    Right_Side = -0.9;
    Left_Side = 0.0;
    break;
  case 7:     // slow straight (walls)
    Right_Side = 0.3;
    Left_Side = 0.3; 
    break;
  case 8:     // full power (wall)
    Right_Side = 0.8;
    Left_Side = 0.8;
    break; 
  default:
    Right_Side = 0.0;
    Left_Side = 0.0;
    break;
  }
}

 

/*******************************************************************************
* FUNCTION NAME: Process_Data_From_Master_uP
* PURPOSE:       Executes every 17ms when it gets new data from the master 
*                microprocessor.
* CALLED FROM:   main.c
* ARGUMENTS:     none
* RETURNS:       void
*******************************************************************************/
void Process_Data_From_Master_uP(void)
{
  int left_light, right_light;
  int left_prox, middle_prox, right_prox;
  int limit_lower, limit_upper;
  float diff_light, diff_prox;
  int SI_out, pixel_out, pixel_in;
  
  Getdata(&rxdata);   /* Get fresh data from the master microprocessor. */

  Default_Routine();  /* Processes joystick commands. */


  /* Get sensor input values. */
  left_light = (int)Get_Analog_Value(rc_ana_in02);
  right_light = (int)Get_Analog_Value(rc_ana_in01);
  diff_light = Set_L_Light_Sensor(left_light) - Set_R_Light_Sensor(right_light);

  left_prox = (int)Get_Analog_Value(rc_ana_in06);
  middle_prox = (int)Get_Analog_Value(rc_ana_in07);
  right_prox = (int)Get_Analog_Value(rc_ana_in05);
  diff_prox = Set_L_Prox(left_prox) - Set_R_Prox(right_prox);
  
  limit_lower = (int)Get_Analog_Value(rc_ana_in04);
  limit_upper = (int)Get_Analog_Value(rc_ana_in03);
  
  
  // run camera code every iteration??
  SI_out = 1;
  rc_dig_out16 = SI_out;
  SI_out = 0;
  rc_dig_out16 = SI_out;
  for(int i = 0; i < 128; i++) {
    rc_dig_out14 = 1;
    rc_dig_out14 = 0;
  }
  pixel_in = (int)Get_Analog_Value(rc_ana_in08);
  println("Camera Value: %d", pixel_in);
  
  
  if(counter > 0) {    // persistent turn mode 
    // maintain old drive_state and auto_mode
    counter = counter - 1;
    Process_Driving_State(drive_state);
  }
  else {


  /*  Autonomous cases  */
  switch(auto_mode) {
    case 0:
      // joystick mode, do nothing
      break;
      
    case 1:      // light source following
      if (light_count > 0) { light_count = light_count - 1; }
      
      if (left_light > 960 && right_light > 960) {
        // spinning search mode
        drive_state = 4;               /////////// check on race day ///////////
      }
      else if (diff_light > 0.33) {  // right
        drive_state = 3;
      }
      else if (diff_light < -0.33) {  // left
        drive_state = 4;
      }
      else {      // straight
        drive_state = 1;
      }

      // end condition
      if (middle_prox > 170 && light_count == 0) {
        drive_state = 0;
        arm_count = period;
        auto_mode = 4;
      }
      
      Process_Driving_State(drive_state);
      break;
      
    case 2:    // avoid walls mode
      if (diff_prox > 0.3) {  // right
        drive_state = 3;
      }
      else if (diff_prox < -0.35) {  // left
        drive_state = 4;
      }
      else {     // straight
        drive_state = 1;
      }
      
      if (middle_prox > 70) {
        if (diff_prox > 0.0) {
          drive_state = 3; 
        }      // right
        else {
          drive_state = 4;
        }       // left
      }
      
      if (left_prox < 50 && right_prox < 50) {   // end condition
        auto_mode = 1; 
        light_count = 200;
      }
      
      Process_Driving_State(drive_state);
      break;
      
    case 3:    // line tracker mode
      // 
      drive_state = 0;
      
      if (middle_prox > 150) {  // end condition
        drive_state = 0;
        auto_mode = 5;
      }  // enter wall climb mode
      
      Process_Driving_State(drive_state);
      break;
          
    case 4:      // lower arm, counter, hand closed      
      if (arm_count > 0) {
        hand_pwm = closed;
        arm_pwm = 0;
        arm_count = arm_count - 1;
      } else { 
        hand_pwm = open;
        arm_pwm = 127;
        auto_mode = 0; 
      }
      
      if (limit_lower < 500) {
        arm_count = 0;
      }
      drive_state = 0;
      Process_Driving_State(drive_state);
      break;
      
    case 5:     // wall climber,assume front first
      if (middle_prox < 150) {
        drive_state = 1;     //post wall
        if (left_prox > 15 || right_prox > 15)
        { auto_mode = 2; }     // end condition
      }
      else if (middle_prox > 150 && middle_prox < 400)
      { drive_state = 7; }
      else if (middle_prox > 400) {
        drive_state = 8;    // full speed over
        counter = 250;
      } // maybe use back_prox instead
      
      Process_Driving_State(drive_state);
      break;

    default:
      drive_state = 0;
      Process_Driving_State(drive_state);
      break;
       
  }     // end of switch
  }


  if (auto_mode == 0)
  { slow_mode = 1; }    //seems good for joystick mode
  else
  { slow_mode = 1; }
  divisor = 4.0 * ((float)(slow_mode + 1));

  if (limit_lower < 500 && arm_pwm < 127)   // stop arm
  { arm_pwm = 127; }
  if (limit_upper > 500 && arm_pwm > 127)
  { arm_pwm = 127; }

//  printf("Chute: Left = %d, Middle = %d, Right = %d\n", 
//      left_prox,middle_prox,right_prox);
//  printf("left= %d, right= %d, prox= %d\n", 
//      left_light,right_light,middle_prox);
//  printf("auto_mode= %d, count= %d\n",auto_mode,btn_count);



  // four wheel drive        // 3,4,5,6 reverse
  pwm06 = Set_LB_Motor(Left_Side);
  pwm05 = Set_RB_Motor(Right_Side);
  pwm04 = Set_LF_Motor(Left_Side);
  pwm03 = Set_RF_Motor(Right_Side);
    
  // arm and hand control
  pwm02 = arm_pwm;
  pwm07 = hand_pwm;
 
  Putdata(&txdata);             /* DO NOT CHANGE! */
}


/*******************************************************************************
* FUNCTION NAME: Default_Routine
* PURPOSE:       Performs the default mappings of inputs to outputs for  
*                CONFIGURATIONS A, B, and C (B = drive mode 12 - settings on radio)
* CALLED FROM:   this file, Process_Data_From_Master_uP routine
* ARGUMENTS:     none
* RETURNS:       void
*******************************************************************************/
void Default_Routine(void)
{
  // Wheel and Driving control 
  Right_Side = -(((float)Limit_Mix(2000 + PWM_in1 + PWM_in2 - 127)) / 255 - 0.5) * 2.0;
  Left_Side = -(((float)Limit_Mix(2000 + PWM_in2 - PWM_in1 + 127)) / 255 - 0.5) * 2.0;
  
  // buffer for complete stop, [122,132]
  if (Right_Side < -0.05) { 
    Right_Side = Right_Side + 0.05;
  } else if (Right_Side > 0.05) {
    Right_Side = Right_Side - 0.05;
  } else {
    Right_Side = 0.0;
  }
  // buffer for complete stop, [122,132]
  if (Left_Side < -0.05) { 
    Left_Side = Left_Side + 0.05;
  } else if (Left_Side > 0.05) {
    Left_Side = Left_Side - 0.05;
  } else {
    Left_Side = 0.0;
  }

  // prevent counterrotating wheels
  if (Left_Side < 0.0 && Right_Side > 0.0)
  { Right_Side = 0.0; }
  if (Right_Side < 0.0 && Left_Side > 0.0)
  { Left_Side = 0.0; }


  // arm control  
  arm_pwm = PWM_in3;
  if (arm_pwm > 170) { arm_pwm = 255; } 
  else if (arm_pwm < 90) { arm_pwm = 0; } 
  else { arm_pwm = 127; }   // buffer zone

    
  // Debounce buttons
  if (btn_count > 100) { btn_count = 0; }
   
  //Handle Channel 5 receiver button
  if (PWM_in5 < BUTTON_REV_THRESH) {
    btn_count = btn_count + 1;
    if (btn_count == 5) {
      if (auto_mode == 0) { auto_mode = 5; }
      else { auto_mode = auto_mode - 1; }
       
      if (auto_mode == 4) { arm_count = period; }
      if (auto_mode == 1) { light_count = 0; }
    }
  } else if (PWM_in5 > BUTTON_FWD_THRESH) {
    btn_count = btn_count + 1;
    if (btn_count == 5) {
      if (auto_mode == 5) { auto_mode = 0; }
      else { auto_mode = auto_mode + 1; }
       
      if (auto_mode == 4) { arm_count = period; }
      if (auto_mode == 1) { light_count = 0; }
    }
  } else { 
    btn_count = 0;
  }

  if (btn_count2 > 100) { btn_count2 = 0; }
  //Handle Channel 6 receiver button
  if (PWM_in6 < BUTTON_REV_THRESH) {
    btn_count2 = btn_count2 + 1;
 if (btn_count2 == 5) {
      hand_pwm = open;
    }
  } else if (PWM_in6 > BUTTON_FWD_THRESH) {
    btn_count2 = btn_count2 + 1;
    if (btn_count2 == 5) {
      hand_pwm = closed;
    }
  } else {
    btn_count2 = 0;
  }
  
}
/******************************************************************************/
