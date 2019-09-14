/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/


package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWMTalonSRX;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;
import edu.wpi.first.cameraserver.CameraServer;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.VictorSP;
import static frc.robot.Constants.*;





/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  //autonomuous boilerplate
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  //init drivetrain
  WPI_TalonSRX leftFront = new WPI_TalonSRX(LEFT_FRONT_DRIVE_CONTROLLER_ID);
  WPI_VictorSPX leftBack = new WPI_VictorSPX(LEFT_BACK_DRIVE_CONTROLLER_ID);
  WPI_VictorSPX rightFront = new WPI_VictorSPX(RIGHT_FRONT_DRIVE_CONTROLLER_ID);
  WPI_VictorSPX rightBack = new WPI_VictorSPX(RIGHT_BACK_DRIVE_CONTROLLER_ID);

  SpeedControllerGroup leftSide = new SpeedControllerGroup(leftFront, leftBack);
  SpeedControllerGroup rightSide = new SpeedControllerGroup(rightFront, rightBack);
  DifferentialDrive robotDrive = new DifferentialDrive(leftSide,rightSide);

  //init CANSpark
  CANSparkMax m_motor = new CANSparkMax(ARM_MOTOR_SPARK_CONTROLLER_ID,MotorType.kBrushless);
  CANEncoder encoder;

  //init PWM motor controllers
  PWMTalonSRX intake = new PWMTalonSRX(INTAKE_PWM_CH);

  //init pneumatics
  Compressor c = new Compressor(PCM_ID);
  Solenoid gripper = new Solenoid(GRIPPER_PCM_CH);
  Solenoid yoshi = new Solenoid(YOSHI_PCM_CH);
  Solenoid boom = new Solenoid(BOOM_PCM_CH);
  
  


  //init controls
  private final Joystick m_stick = new Joystick(JOYSTICK_PORT);
  


  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    CameraServer.getInstance().startAutomaticCapture();

    encoder = m_motor.getEncoder();
    m_motor.restoreFactoryDefaults();

    //CTRE Config
    leftFront.configFactoryDefault(0);
    leftFront.configOpenloopRamp(RAMP_TIME);
    leftBack.configFactoryDefault(0);
    leftBack.configOpenloopRamp(RAMP_TIME);
    rightFront.configFactoryDefault(0);
    rightFront.configOpenloopRamp(RAMP_TIME);
    leftBack.configFactoryDefault(0);
    leftBack.configOpenloopRamp(RAMP_TIME);


    //Pneumatics
    //c.setClosedLoopControl(true);
    
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
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    double _speed = -1*m_stick.getRawAxis(SPEED_AXIS)*SPEED_MAX;
    double _turn = m_stick.getRawAxis(TURN_AXIS)*TURN_MAX;
    robotDrive.arcadeDrive(_speed, _turn);
    

    //Intake actions
    double intakeSpeed = m_stick.getRawAxis(INTAKE_AXIS);
    double outtakeSpeed = m_stick.getRawAxis(OUTTAKE_AXIS);
    intake.set(intakeSpeed-outtakeSpeed);
    
    //Solenoid actions
    gripper.set(m_stick.getRawButton(GRIPPER_BTN));
    yoshi.set(m_stick.getRawButton(YOSHI_BTN));
    boom.set(m_stick.getRawButton(BOOM_BTN)); //should be called

    double motorSpeed = 0.5*(m_stick.getRawAxis(ARM_AXIS));
    m_motor.set(motorSpeed);

  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}

