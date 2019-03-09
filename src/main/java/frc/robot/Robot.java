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
  private Joystick leftDriverStick = new Joystick(0);
  private Joystick rightDriverStick = new Joystick(1);
  private Joystick leftOperatorStick = new Joystick(2);
  private Joystick rightOperatorStick = new Joystick(3);

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

  // Solenoids for pneumatics to follow...
  private DoubleSolenoid armShortTelescopeSolenoid = new DoubleSolenoid(kPCMID, 0, 1);
  private DoubleSolenoid armDeploySolenoid = new DoubleSolenoid(kPCMID, 2, 3);
  private DoubleSolenoid armLongTelescopeSolenoid = new DoubleSolenoid(kPCMID, 4, 5);
  private DoubleSolenoid armGrabSolenoid = new DoubleSolenoid(kPCMID, 6, 7);
  private DoubleSolenoid armPunchSolenoid = new DoubleSolenoid(kPCMID, 8, 9);
  private Solenoid armUnlockSolenoid = new Solenoid(kPCMID, 10);

  // Misc. objects
  private final Object imgLock = new Object();
  private enum MyRobotState {
    DISABLED,
    AUTONOMOUS_BEGIN,
    AUTONOMOUS_TILT,
    AUTONOMOUS_TELESCOPE,
    AUTONOMOUS_DEPLOY,
    ROBOT_READY,
    END_BEGIN,
    END_UNLOCK,
    END_UNDEPLOY,
    END_UNTELESCOPE,
    ROBOT_END,
  };
  private MyRobotState robotState = MyRobotState.DISABLED;
  private Timer timer = new Timer();
  private boolean isHolding = false;

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

    // Set all DoubleSolenoids to STARTING positions. FIXME before competition
    //armShortTelescopeSolenoid.set(Value.kReverse);
    //armDeploySolenoid.set(Value.kReverse);
    //armLongTelescopeSolenoid.set(Value.kReverse);
    //armGrabSolenoid.set(Value.kReverse);
    //armPunchSolenoid.set(Value.kReverse);
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
    robotState = MyRobotState.AUTONOMOUS_BEGIN;
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    if (robotState == MyRobotState.ROBOT_READY) {
      // If the arm is ready, then simply use human controls.
      teleopPeriodic();
    } else {
      // Arm is not ready -- the following method can be used
      handleAutonomousStateUpdate();
    }
  }

  private void handleAutonomousStateUpdate() {
    if (robotState == MyRobotState.AUTONOMOUS_BEGIN) {
      timer.reset();
      timer.start();
      // Start tilting to level the arm
      armTiltMotorController.set(kMaxPower);
      robotState = MyRobotState.AUTONOMOUS_TILT;
    } else if (robotState == MyRobotState.AUTONOMOUS_TILT) {
      // We wait .1 second of tilting and assume arm is relatively level
      if (timer.get() > 0.1) {
        // Once .1 second has passed, do the next thing
        timer.stop();
        armTiltMotorController.set(0.0);
        robotState = MyRobotState.AUTONOMOUS_TELESCOPE;
      }
    } else if (robotState == MyRobotState.AUTONOMOUS_TELESCOPE) {
      // armShortTelescopeSolenoid.set(Value.kForward);
      robotState = MyRobotState.AUTONOMOUS_DEPLOY;
    } else if (robotState == MyRobotState.AUTONOMOUS_DEPLOY) {
      // armDeploySolenoid.set(Value.kForward);
      robotState = MyRobotState.ROBOT_READY;
    } else if (robotState == MyRobotState.ROBOT_READY) {
      // Everything is already done
      return;
    } else {
      // We have no idea what is happening. Set to AUTONOMOUS_BEGIN and hope that fixes it
      // This could have the unfortunate effect of causing an infinite loop.
      robotState = MyRobotState.AUTONOMOUS_BEGIN;
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
    armLongTelescope();
    armGrabAndPunch();
    manualDeploy();
    handleTeleopStateUpdate();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  /**
   * Driver sticks only control tank drive
   */
  private void tankDrive() {
    // Get the position of each joystick in the vertical (up-down) axiss
    double leftStickPower = -1.0 * kMaxPower * leftDriverStick.getRawAxis(kVerticalAxis);
    double rightStickPower = kMaxPower * rightDriverStick.getRawAxis(kVerticalAxis);

    // Set both left motors to the amount of power on the left stick.
    leftTankMotor1Controller.set(leftStickPower);
    leftTankMotor2Controller.set(leftStickPower);
    // Set both right motors to the amount of power on the right stick.
    rightTankMotor1Controller.set(rightStickPower);
    rightTankMotor2Controller.set(rightStickPower);
  }

  /**
   * Use left operator stick Y axis for controlling front lift.
   */
  private void frontLift() {
    double frontLiftPower = kMaxPower * leftOperatorStick.getRawAxis(kVerticalAxis);
    frontLiftMotor1Controller.set(frontLiftPower);
    frontLiftMotor2Controller.set(frontLiftPower);
  }

  /**
   * Use left operator buttons to control spider wheels:
   *   * trigger button moves forward
   *   * right button moves backwards (rarely used)
   */
  private void spiderWheels() {
    // if trigger button pressed
    if (leftOperatorStick.getRawButton(1)) {
      // spider wheels forward
      spiderWheelMotor1Controller.set(1.0*kMaxPower);
      spiderWheelMotor2Controller.set(1.0*kMaxPower);
    } else if (leftOperatorStick.getRawButton(5)) {
      spiderWheelMotor1Controller.set(-1.0*kMaxPower);
      spiderWheelMotor2Controller.set(-1.0*kMaxPower);
    } else {
      // else stop
      spiderWheelMotor1Controller.set(0);
      spiderWheelMotor2Controller.set(0);
    }
  }

  /**
   * Use either left or right operator stick buttons to control rear lift
   *   * center button lifts (foot goes down, robot goes up)
   *   * back button descends (foot goes up, robot goes down)
   */
  private void rearLift() {
    if (leftOperatorStick.getRawButton(3) || rightOperatorStick.getRawButton(3)) {
      // lift
      rearLiftMotorController.set(1.0*kMaxPower);
    } else if (leftOperatorStick.getRawButton(2) || rightOperatorStick.getRawButton(2)) {
      // descend
      rearLiftMotorController.set(-1.0*kMaxPower);
    } else {
      // neither
      rearLiftMotorController.set(0);
    }
  }

  /**
   * Use right operator stick Y axis for controlling arm tilt.
   */
  private void armTilt() {
    // get where axis is times 20%
    double armLiftPower = kMaxPower * rightOperatorStick.getRawAxis(kVerticalAxis);
 
    // TODO: prevent moving beyond 45deg tilt using switch
 
    // set power of talon to axis
    armTiltMotorController.set(armLiftPower);
  }

  /**
   * Use right operator buttons to control extension
   *   * left button to extend
   *   * right button to retract
   */
  private void armLongTelescope() {
    if(rightOperatorStick.getRawButton(4)){
      // extend
      //armLongTelescopeSolenoid.set(Value.kForward);
    }else if(rightOperatorStick.getRawButton(5)){
      // retract
      //armLongTelescopeSolenoid.set(Value.kReverse);
    }else{
      // do nothing
    }
  }

  /**
   * Use right operator stick trigger button for grab/punch.
   * 
   * Operator holds button to hold ball
   * When ball released, then punch
   */
  private void armGrabAndPunch() {
    if (rightOperatorStick.getRawButton(1)) {
      // armGrabSolenoid.set(Value.kForward);
    } else if (isHolding) {
      // armGrabSolenoid.set(Value.kReverse);
      // armPunchSolenoid.set(Value.kForward);
      isHolding = false;
    } else {
      // armGrabSolenoid.set(Value.kReverse);
      // armPunchSolenoid.set(Value.kReverse);
    }
  }

  private void manualDeploy() {
    // controls the things for arm deployment in case of failure of autonomous/end modes
    if (leftOperatorStick.getRawButton(8)) {
      armShortTelescopeSolenoid.set(Value.kForward);
    } else if (leftOperatorStick.getRawButton(9)) {
      armShortTelescopeSolenoid.set(Value.kReverse);
    }
    if (leftOperatorStick.getRawButton(10)) {
      armDeploySolenoid.set(Value.kForward);
    } else if (leftOperatorStick.getRawButton(11)) {
      armDeploySolenoid.set(Value.kReverse);
    }
  }

  private void handleTeleopStateUpdate() {
    // Handle end sequence for robot
    // Start with how to start sequence -- press bottom button on rightOperatorStick
    if (rightOperatorStick.getRawButton(6)) {
      robotState = MyRobotState.END_BEGIN;
    } // else if  (...) chain TODO
  }

  private void stopRobot() {
    robotState = MyRobotState.DISABLED;
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
    isHolding = false;    
  }
}