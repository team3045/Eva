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
import edu.wpi.first.wpilibj.Joystick;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private Joystick leftStick = new Joystick(0);
  private Joystick rightStick = new Joystick(1);
  // IMPORTANT: On the real robot, you want to make sure these numbers are correct.
  // You don't want motors on the same side to spin at different speeds or different direction.
  private WPI_TalonSRX leftMotor1Controller = new WPI_TalonSRX(5);
  private WPI_TalonSRX leftMotor2Controller = new WPI_TalonSRX(6);
  private WPI_TalonSRX leftMotor3Controller = new WPI_TalonSRX(7);
  private WPI_TalonSRX rightMotor1Controller = new WPI_TalonSRX(0);
  private WPI_TalonSRX rightMotor2Controller = new WPI_TalonSRX(1);
  private WPI_TalonSRX rightMotor3Controller = new WPI_TalonSRX(2);

  /**\
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
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
    tankDrive();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  private void tankDrive() {
    // This assumes the vertical axis is number 1. In reality this depends on the joystick.
    int kVerticalAxis = 1;
    // Cap power to a smaller amount
    double kMaxPower = 0.2;
    // Get the position of each joystick in the vertical (up-down) axiss
    double leftStickPower = kMaxPower * leftStick.getRawAxis(kVerticalAxis);
    double rightStickPower = -1.0 * kMaxPower * rightStick.getRawAxis(kVerticalAxis);

    // Set both left motors to the amount of power on the left stick.
    leftMotor1Controller.set(leftStickPower);
    leftMotor2Controller.set(leftStickPower);
    leftMotor3Controller.set(leftStickPower);
    // Set both right motors to the amount of power on the right stick.
    rightMotor1Controller.set(rightStickPower);
    rightMotor2Controller.set(rightStickPower);
    rightMotor3Controller.set(rightStickPower);
  }
}
