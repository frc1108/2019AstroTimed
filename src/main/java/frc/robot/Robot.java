/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/


package frc.robot;

//wpilib imports
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWMTalonSRX;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;

//ctre imports
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

//rev imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.ControlType;

//other imports
import static frc.robot.Constants.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  
  //init drivetrain
  private WPI_TalonSRX leftFront = new WPI_TalonSRX(LEFT_FRONT_DRIVE_CONTROLLER_ID);
  private WPI_VictorSPX leftBack = new WPI_VictorSPX(LEFT_BACK_DRIVE_CONTROLLER_ID);
  private WPI_VictorSPX rightFront = new WPI_VictorSPX(RIGHT_FRONT_DRIVE_CONTROLLER_ID);
  private WPI_VictorSPX rightBack = new WPI_VictorSPX(RIGHT_BACK_DRIVE_CONTROLLER_ID);

  private SpeedControllerGroup leftSide = new SpeedControllerGroup(leftFront, leftBack);
  private SpeedControllerGroup rightSide = new SpeedControllerGroup(rightFront, rightBack);
  private DifferentialDrive robotDrive = new DifferentialDrive(leftSide,rightSide);
  public boolean isInvertedDrive = false;

  //init Arm SparkMAX
  private CANSparkMax m_motor = new CANSparkMax(ARM_MOTOR_SPARK_CONTROLLER_ID,MotorType.kBrushless);
  private CANEncoder m_encoder;
  private CANPIDController m_pidController;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, kZeroPosition, maxVel, minVel, maxAcc, allowedErr, setPoint;

  //init Intake
  private DigitalInput m_intakeLoaded = new DigitalInput(INTAKE_DIO_CH);
  private PWMTalonSRX intake = new PWMTalonSRX(INTAKE_PWM_CH);
  private Timer t = new Timer();
  public boolean isOutakePressed = false;
  public boolean isOutakeReleased = false;

  //init level 2
  private Spark levelClimber = new Spark(CLIMB_PWM_CH);

  //init pneumatics
  private Compressor c = new Compressor(PCM_ID);
  private Solenoid gripper = new Solenoid(GRIPPER_PCM_CH);
  private Solenoid yoshi = new Solenoid(YOSHI_PCM_CH);
  private Solenoid boom = new Solenoid(BOOM_PCM_CH);
  private Solenoid front = new Solenoid(FRONT_PCM_CH);
  private Solenoid back = new Solenoid(BACK_PCM_CH);

  
  //init controls
  private Joystick m_stick = new Joystick(JOYSTICK_PORT);
  private Joystick m_operator = new Joystick(OPERATOR_PORT);


  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    driveConfig();
    armMotionConfig();
    t.start();
      
  }
  
  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    }
  

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    

    if(m_stick.getRawButtonPressed(INVERT_BTN)){
      isInvertedDrive = !isInvertedDrive;
    }

    double _speed = -1*(isInvertedDrive?1:-1)*m_stick.getRawAxis(SPEED_AXIS)*SPEED_MAX;
    double _turn = (isInvertedDrive?1:-1)*m_stick.getRawAxis(TURN_AXIS)*TURN_MAX;
    robotDrive.arcadeDrive(_speed, _turn);
    

    //Intake sensor
    SmartDashboard.putBoolean("Intake Loaded", !m_intakeLoaded.get());


    //Intake actions
    double intakeSpeed = !m_intakeLoaded.get()?0:m_stick.getRawAxis(INTAKE_AXIS);
    isOutakePressed = m_stick.getRawAxis(OUTTAKE_AXIS)>0.2;
    if (isOutakePressed) {

    }
    isOutakeReleased = !isOutakePressed;
    double outtakeSpeed = m_stick.getRawAxis(OUTTAKE_AXIS);
    intake.set(intakeSpeed-outtakeSpeed);


    
    //Solenoid actions
    //gripper release cargo when pressed
    gripper.set(m_stick.getRawButton(GRIPPER_BTN));
    //yoshi toggle
    if(m_stick.getRawButtonPressed(YOSHI_BTN)){
      yoshi.set(!yoshi.get());
    }
    
    //boom toggle
    if(m_stick.getRawButtonPressed(BOOM_BTN)){
      boom.set(!boom.get());
    }

    //climb
    if(m_operator.getRawButtonPressed(YOSHI_BTN)){
      front.set(!front.get());
      levelClimber.set((levelClimber.get()>0.5)?0:1);
    }
    if(m_operator.getRawButtonPressed(YOSHI_BTN)){
      front.set(false);
      levelClimber.set(0);
      back.set(!back.get());
    }
    
    //arm movement
    setPoint = (m_stick.getRawAxis(ARM_AXIS)>0.5)?15.0:5.0;

    
    //double motorSpeed = 0.4*(m_stick.getRawAxis(ARM_AXIS));
    //m_motor.set(motorSpeed);
    armMotionUpdate();

  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  /** 
   * Setup of drive motor controllers
   */
  public void driveConfig(){
    //CTRE Config
    leftFront.configFactoryDefault(0);
    leftFront.configOpenloopRamp(RAMP_TIME);
    leftBack.configFactoryDefault(0);
    leftBack.configOpenloopRamp(RAMP_TIME);
    rightFront.configFactoryDefault(0);
    rightFront.configOpenloopRamp(RAMP_TIME);
    leftBack.configFactoryDefault(0);
    leftBack.configOpenloopRamp(RAMP_TIME);
  }
/**
 * Setup smart motion of arm SparkMax 
 */
  public void armMotionConfig(){
      /**
     * The RestoreFactoryDefaults method can be used to reset the configuration parameters
     * in the SPARK MAX to their factory default state. If no argument is passed, these
     * parameters will not persist between power cycles
     */
    m_motor.restoreFactoryDefaults();

    // initialze PID controller and encoder objects
    m_pidController = m_motor.getPIDController();
    m_encoder = m_motor.getEncoder();

    // PID coefficients
    kP = 0.000125; 
    kI = 0;
    kD = 0.0000625; 
    kIz = 0; 
    kFF = 0.0015; 
    kMaxOutput = 1; 
    kMinOutput = -1;
    kZeroPosition = 0;
    allowedErr = 0.001;
    maxRPM = 5700;

    // Smart Motion Coefficients
    maxVel = 3000; // rpm default 2000:1500
    maxAcc = 2000;

    // set PID coefficients
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);
    m_encoder.setPosition(kZeroPosition);
    

    /**
     * Smart Motion coefficients are set on a CANPIDController object
     * 
     * - setSmartMotionMaxVelocity() will limit the velocity in RPM of
     * the pid controller in Smart Motion mode
     * - setSmartMotionMinOutputVelocity() will put a lower bound in
     * RPM of the pid controller in Smart Motion mode
     * - setSmartMotionMaxAccel() will limit the acceleration in RPM^2
     * of the pid controller in Smart Motion mode
     * - setSmartMotionAllowedClosedLoopError() will set the max allowed
     * error for the pid controller in Smart Motion mode
     */
    int smartMotionSlot = 0;
    m_pidController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    m_pidController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
    m_pidController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    m_pidController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);
  }

  private void armMotionUpdate(){
     /**
      * As with other PID modes, Smart Motion is set by calling the
      * setReference method on an existing pid object and setting
      * the control type to kSmartMotion
      */
     m_pidController.setReference(setPoint, ControlType.kSmartMotion);
         
 
  }

}

