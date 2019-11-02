/**
 * Constants for Robot 
 */




 package frc.robot;


 public class Constants{
    //CAN IDs
    public final static int LEFT_FRONT_DRIVE_CONTROLLER_ID = 0;
    public final static int LEFT_BACK_DRIVE_CONTROLLER_ID = 1;
    public final static int RIGHT_FRONT_DRIVE_CONTROLLER_ID = 2;
    public final static int RIGHT_BACK_DRIVE_CONTROLLER_ID = 3;
    public final static int ARM_MOTOR_SPARK_CONTROLLER_ID = 1;
    
    //OI Joysticks
    public final static int OPERATOR_PORT = 1;
    public final static int DRIVER_PORT = 0;

    //Driver Axis
    public final static int TURN_AXIS = 0;    //left horizontal
    public final static int SPEED_AXIS = 1;   //left vertical
    public final static int INTAKE_AXIS = 2;  //back left
    public final static int OUTTAKE_AXIS = 3; //back right
    public final static int ARM_AXIS_2 = 4; //right horizontal
    public final static int ARM_AXIS = 5;     //right vertical

    //Driver buttons
    public final static int BOOM_BTN = 1;
    public final static int HATCH_BTN = 4;
    public final static int YOSHI_BTN = 5;
    public final static int GRIPPER_BTN = 6;
    public final static int CLIMB_MODE_BTN = 7;
    public final static int BACK_BTN = 8;
    public final static int INVERT_BTN = 9;
    public final static int ARM_MODE_BTN = 10;

    //Drivetrain Characteristics
    public final static double RAMP_TIME = 0.15; //seconds
    public final static double SPEED_MAX = 1;
    public final static double TURN_MAX = 1;
    

    //PCM IDs
    public final static int PCM_ID = 0;

    //PCM Solenoids
    public final static int GRIPPER_PCM_CH = 0;
    public final static int YOSHI_PCM_CH = 1;
    public final static int BOOM_PCM_CH = 2; 
    public final static int BACK_PCM_CH = 4;
    public final static int FRONT_PCM_CH = 5;
    public final static int HATCH_PCM_CH = 6;
    
    //Intake
    public final static int INTAKE_PWM_CH = 0; //flipped output on one motor controller
    public final static int INTAKE_DIO_CH = 9; //flipped output on one motor controller

    //Climber
    public final static int CLIMB_PWM_CH = 3; //flipped output on one motor controller





 } 





 