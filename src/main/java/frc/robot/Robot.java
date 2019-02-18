/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.vision.VisionThread;


import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.cscore.UsbCamera;
import frc.robot.GreenTargetDetector;
import java.util.ArrayList;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import edu.wpi.first.wpilibj.DoubleSolenoid;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  // Joystick setup
  private Joystick leftStick = new Joystick(0);
  private Joystick rightStick = new Joystick(1);
  private Joystick operatorStick1 = new Joystick(2);

  // Defines horizontal and vertical axis of joysticks
  // This assumes the vertical axis is number 1. In reality this depends on the joystick.
  private final int kHorizontalAxis = 0;
  private final int kVerticalAxis = 1;

  // Talon CAN bus ids
  private final int kLeftTankMotor1ID = 6;
  private final int kLeftTankMotor2ID = 7;
  private final int kRightTankMotor1ID = 0;
  private final int kRightTankMotor2ID = 1;
  private final int kFrontLiftMotor1ID = 3;
  private final int kFrontLiftMotor2ID = 4;
  private final int kSpiderWheelMotor1ID = 99;
  private final int kSpiderWheelMotor2ID = 99;
  private final int kRearLiftMotorID = 5;
  private final int kArmTiltMotorID = 99;
  private final int kArmPanMotorID = 99;
  private final int kArmTelescopeMotorID = 99;
  private final int kPCMID = 21;

  // Talons
  private WPI_TalonSRX leftTankMotor1Controller = new WPI_TalonSRX(kLeftTankMotor1ID);
  private WPI_TalonSRX leftTankMotor2Controller = new WPI_TalonSRX(kLeftTankMotor2ID);
  private WPI_TalonSRX rightTankMotor1Controller = new WPI_TalonSRX(kRightTankMotor1ID);
  private WPI_TalonSRX rightTankMotor2Controller = new WPI_TalonSRX(kRightTankMotor2ID);
  private WPI_TalonSRX frontLiftMotor1Controller = new WPI_TalonSRX(kFrontLiftMotor1ID);
  private WPI_TalonSRX frontLiftMotor2Controller = new WPI_TalonSRX(kFrontLiftMotor2ID);
  private WPI_TalonSRX spiderWheelMotor1Controller = new WPI_TalonSRX(kSpiderWheelMotor1ID);
  private WPI_TalonSRX spiderWheelMotor2Controller = new WPI_TalonSRX(kSpiderWheelMotor2ID);
  private WPI_TalonSRX rearLiftMotorController = new WPI_TalonSRX(kRearLiftMotorID);
  private WPI_TalonSRX armTiltMotorController = new WPI_TalonSRX(kArmTiltMotorID);
  private WPI_TalonSRX armPanMotorController = new WPI_TalonSRX(kArmPanMotorID);
  private WPI_TalonSRX armTelescopeMotorController = new WPI_TalonSRX(kArmTelescopeMotorID);

  // Solenoids for pneumatics to follow...
  private DoubleSolenoid testSolenoid = new DoubleSolenoid(kPCMID,0,1);

  // Misc. objects
  private final Object imgLock = new Object();
  private boolean isAutonomousStart = false;
  private Timer timer = new Timer();

  // Cap power to a smaller amount for now
  private final double kMaxPower = 0.2;

  /**\
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    stopRobot();
  
    UsbCamera frontCamera = CameraServer.getInstance().startAutomaticCapture(0);
    frontCamera.setResolution(320, 240);
    frontCamera.setFPS(8);
    UsbCamera rearCamera = CameraServer.getInstance().startAutomaticCapture(1);
    rearCamera.setResolution(320, 240);
    rearCamera.setFPS(8);
    SmartDashboard.putString("Targeting", "Not Targeted");
    
    VisionThread visionThread = new VisionThread(frontCamera, new GreenTargetDetector(), pipeline -> {
      ArrayList<MatOfPoint> greenRectangles = pipeline.filterContoursOutput();
      SmartDashboard.putString("Targeting", "Found " + greenRectangles.size() + " rectangles");
      if (greenRectangles.size() > 1) {
        Rect r = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0));
        synchronized (imgLock) {
          SmartDashboard.putString("Width", ""+r.width);
          SmartDashboard.putString("Height", ""+r.height);
        }
      }
    });
    visionThread.start();

    testSolenoid.set(DoubleSolenoid.Value.kReverse);
  }

  @Override
  public void disabledPeriodic() {
    stopRobot();
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
    isAutonomousStart = true;
    timer.reset();
    timer.start();
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    if (isAutonomousStart && timer.get() < 1.0) {
      // Floor it for a second
      floorIt();
    } else {
      if (isAutonomousStart) {
        timer.stop();
        isAutonomousStart = false;
      }

      // Auton and teleop are the same this year
      teleopPeriodic();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    tankDrive();
    frontLift();
    spiderWheels();
    rearLift();
    armTilt();
    armPan();
    armTelescope();
    testSolenoid();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  private void tankDrive() {
    // Get the position of each joystick in the vertical (up-down) axiss
    double leftStickPower = -1.0 * kMaxPower * leftStick.getRawAxis(kVerticalAxis);
    double rightStickPower = kMaxPower * rightStick.getRawAxis(kVerticalAxis);

    // Set both left motors to the amount of power on the left stick.
    leftTankMotor1Controller.set(leftStickPower);
    leftTankMotor2Controller.set(leftStickPower);
    // Set both right motors to the amount of power on the right stick.
    rightTankMotor1Controller.set(rightStickPower);
    rightTankMotor2Controller.set(rightStickPower);
  }

  private void frontLift() {
    // if button 1 pressed
    if (leftStick.getRawButton(6)) {
      // lift up
      frontLiftMotor1Controller.set(1.0*kMaxPower);
      frontLiftMotor2Controller.set(1.0*kMaxPower);
      // else if button 2 pressed
    } else if (leftStick.getRawButton(7)) {
      // go down
      frontLiftMotor1Controller.set(-1.0*kMaxPower);
      frontLiftMotor2Controller.set(-1.0*kMaxPower);
    } else {
      // if nothing pressed stop
      frontLiftMotor1Controller.set(0);
      frontLiftMotor2Controller.set(0);
    }
  }

  private void spiderWheels() {
    // if button pressed
    if (leftStick.getRawButton(8)) {
      // spider wheels forward
      spiderWheelMotor1Controller.set(1.0*kMaxPower);
      spiderWheelMotor2Controller.set(1.0*kMaxPower);
    } else {
      // else stop
      spiderWheelMotor1Controller.set(0);
      spiderWheelMotor2Controller.set(0);
    }
  }

  private void rearLift() {
    if (rightStick.getRawButton(6)) {
      // up
      rearLiftMotorController.set(1.0*kMaxPower);

    } else if (rightStick.getRawButton(5)) {
      // down
      rearLiftMotorController.set(-1.0*kMaxPower);

    } else {
      // neither
      rearLiftMotorController.set(0);

    }
  }

  private void armTilt() {

    // get where axis is times 20%
    double operatorStick1PowerVertical = kMaxPower * operatorStick1.getRawAxis(kVerticalAxis);
    // set power of talon to axis
    armTiltMotorController.set(operatorStick1PowerVertical);
  }

  private void armPan() {
    // get where axis is times 20%
    double operatorStick1PowerHorizontal = kMaxPower * operatorStick1.getRawAxis(kHorizontalAxis);
    // set power of talon to axis
    armPanMotorController.set(operatorStick1PowerHorizontal);

  }

  private void armTelescope() {
    if(operatorStick1.getRawButton(6)){
      armTelescopeMotorController.set(1.0*kMaxPower);

    }else if(operatorStick1.getRawButton(7)){
      armTelescopeMotorController.set(-1.0*kMaxPower);

    }else{
      armTelescopeMotorController.set(0);
    }
    
  }
  private void testSolenoid(){
    if(leftStick.getRawButton(1)){
      testSolenoid.set(DoubleSolenoid.Value.kForward);
    }else{
      testSolenoid.set(DoubleSolenoid.Value.kReverse);
    }
  }

  private void floorIt() {
    leftTankMotor1Controller.set(-1.0);
    leftTankMotor2Controller.set(-1.0);
    rightTankMotor1Controller.set(1.0);
    rightTankMotor2Controller.set(1.0);
  }

  private void stopRobot() {
    leftTankMotor1Controller.set(0.0);
    leftTankMotor2Controller.set(0.0);
    rightTankMotor1Controller.set(0.0);
    rightTankMotor2Controller.set(0.0);
    frontLiftMotor1Controller.set(0.0);
    frontLiftMotor2Controller.set(0.0);
    spiderWheelMotor1Controller.set(0.0);
    spiderWheelMotor2Controller.set(0.0);
    rearLiftMotorController.set(0.0);
    armTiltMotorController.set(0.0);
    armPanMotorController.set(0.0);
    armTelescopeMotorController.set(0.0);
  }
}
