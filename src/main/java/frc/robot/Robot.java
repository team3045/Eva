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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SpeedControllerGroup;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.cscore.UsbCamera;
import frc.robot.GreenTargetDetector;
import java.util.ArrayList;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;



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
  private final int kLeftTankMotor1ID = 10;
  private final int kLeftTankMotor2ID = 11;
  private final int kRightTankMotor1ID = 12;
  private final int kRightTankMotor2ID = 13;
  private final int kFrontLiftMotor1ID = 14;
  private final int kFrontLiftMotor2ID = 15;
  private final int kSpiderWheelMotor1ID = 16;
  private final int kSpiderWheelMotor2ID = 17;
  private final int kRearLiftMotorID = 18;
  private final int kArmTiltMotorID = 19;
  private final int kPCM1ID = 21;
  private final int kPCM2ID = 22;

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

  private SpeedControllerGroup leftDriveGroup = new SpeedControllerGroup(leftTankMotor1Controller, leftTankMotor2Controller);
  private SpeedControllerGroup rightDriveGroup = new SpeedControllerGroup(rightTankMotor1Controller, rightTankMotor2Controller);
  private DifferentialDrive myDrive = new DifferentialDrive(leftDriveGroup, rightDriveGroup);

  // Solenoids for pneumatics to follow...
  private Solenoid armGrabSolenoid = new Solenoid(kPCM2ID, 0);
  private Solenoid armPunchSolenoid = new Solenoid(kPCM2ID, 1);
  private Solenoid armShortTelescopeSolenoid = new Solenoid(kPCM2ID, 2);
  private DoubleSolenoid armLongTelescopeSolenoid = new DoubleSolenoid(kPCM1ID, 0, 1);
  private DoubleSolenoid armDeploySolenoid = new DoubleSolenoid(kPCM1ID, 3, 2);

  // Misc. objects
  private final Object imgLock = new Object();
  private enum MyRobotState {
    DISABLED,
    AUTONOMOUS_BEGIN,
    AUTONOMOUS_TILT,
    AUTONOMOUS_TELESCOPE,
    ROBOT_READY,
    END_BEGIN,
    END_UNLOCK,
    END_UNDEPLOY,
    END_UNTELESCOPE,
    ROBOT_END,
  };
  private MyRobotState robotState = MyRobotState.DISABLED;
  private Timer timer = new Timer();
  private DigitalInput tiltLimiter = new DigitalInput(0);
  private boolean isTiltLimited = false;
  private boolean isClimbing = false;

  private static final String kTankDrive = "Tank Drive";
  private static final String kArcadeDrive = "Arcade Drive";
  private static final String kCheesyDrive = "Cheesy Drive";
  private final SendableChooser<String> chooser = new SendableChooser<>();

  private final int CAMERA_WIDTH = 320;
  private final int CAMERA_CENTER = CAMERA_WIDTH / 2;
  private final int CAMERA_HEIGHT = 240;

  /**\
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    leftDriveGroup.setInverted(true);
    rightDriveGroup.setInverted(true);

    stopRobot();

    UsbCamera frontCamera = CameraServer.getInstance().startAutomaticCapture(0);
    frontCamera.setResolution(CAMERA_WIDTH, CAMERA_HEIGHT);
    frontCamera.setFPS(4);
    UsbCamera rearCamera = CameraServer.getInstance().startAutomaticCapture(1);
    rearCamera.setResolution(CAMERA_WIDTH, CAMERA_HEIGHT);
    rearCamera.setFPS(4);
    SmartDashboard.putString("Targeting", "No");

    /*
    VisionThread visionThread = new VisionThread(rearCamera, new GreenTargetDetector(), pipeline -> {
      ArrayList<MatOfPoint> greenRectangles = pipeline.filterContoursOutput();
      if (greenRectangles.size() == 2) {
        int center1, center2;
        synchronized (imgLock) {
          if (greenRectangles.size() != 2) {
            return;
          }
          Rect r1 = Imgproc.boundingRect(greenRectangles.get(0));
          Rect r2 = Imgproc.boundingRect(greenRectangles.get(1));
          if (r1 == null || r2 == null) {
            return;
          }
          center1 = r1.x + (r1.width / 2);
          center2 = r2.x + (r2.width / 2);
        }
        if (center1 > center2) {
          int temp;
          temp = center2;
          center2 = center1;
          center1 = temp;
        }
        int dist1 = Math.abs(CAMERA_CENTER - center1);
        int dist2 = Math.abs(CAMERA_CENTER - center2);
        boolean isCentered = Math.abs(dist1-dist2) <= 15;
        if (isCentered) {
          SmartDashboard.putString("Targeting", "Centered");
        } else if (dist1 > dist2) {
          SmartDashboard.putString("Targeting", "Go Left");
        } else {
          SmartDashboard.putString("Targeting", "Go Right");
        }
      } else {
        SmartDashboard.putString("Targeting", "No");
      }
    });
    visionThread.start();
    */
  
    // Set all DoubleSolenoids to STARTING positions. FIXME before competition
    armGrabSolenoid.set(false);
    armPunchSolenoid.set(false);
    armShortTelescopeSolenoid.set(false);
    armLongTelescopeSolenoid.set(Value.kReverse);
    armDeploySolenoid.set(Value.kReverse);

    // Two kinds of controls
    chooser.setDefaultOption("Tank Drive", kTankDrive);
    chooser.addOption("Arcade Drive", kArcadeDrive);
    chooser.addOption("Cheesy Drive", kCheesyDrive);
    SmartDashboard.putData("Drive", chooser);
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
    teleopPeriodic();
  }

  private void handleAutonomousStateUpdate() {
    if (robotState == MyRobotState.AUTONOMOUS_BEGIN) {
      timer.reset();
      timer.start();
      // Start tilting to level the arm
      armTiltMotorController.set(-0.5);
      robotState = MyRobotState.AUTONOMOUS_TILT;
    } else if (robotState == MyRobotState.AUTONOMOUS_TILT) {
      // We wait .1 second of tilting and assume arm is relatively level
      if (timer.get() > 0.5) {
        // Once .1 second has passed, do the next thing
        timer.stop();
        armTiltMotorController.set(0.0);
        robotState = MyRobotState.AUTONOMOUS_TELESCOPE;
      }
    } else if (robotState == MyRobotState.AUTONOMOUS_TELESCOPE) {
      armShortTelescopeSolenoid.set(true);
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
    //armTilt();
    //armLongTelescope();
    //armGrab();
    //armPunch();
    //armDeploy();
    //manualDeploy();
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
    String selectedDrive = chooser.getSelected();
    if (selectedDrive == kArcadeDrive) {
      myDrive.arcadeDrive(rightDriverStick.getRawAxis(kVerticalAxis), -1.0 * rightDriverStick.getRawAxis(kHorizontalAxis));
    } else if (selectedDrive == kCheesyDrive) {
      myDrive.curvatureDrive(rightDriverStick.getRawAxis(kVerticalAxis), -1.0 * rightDriverStick.getRawAxis(kHorizontalAxis), rightDriverStick.getRawButton(1));
    } else { // Tank Drive
      myDrive.tankDrive(leftDriverStick.getRawAxis(kVerticalAxis), rightDriverStick.getRawAxis(kVerticalAxis));
    }
  }

  private static double smoothen(double input) {
    return (input < 0.0 ? -1.0 : 1.0) * Math.pow(input, 2);
  }

  /**
   * Use left operator stick Y axis for controlling front lift.
   */
  private void frontLift() {
    double frontLiftPower;
    double amt = leftOperatorStick.getRawAxis(kVerticalAxis);
    if (amt < 0.0) {
      double newAmt = smoothen(smoothen(amt));
      boolean newIsClimbing = newAmt < -0.15;
      if (newIsClimbing && !isClimbing) {
        timer.reset();
        timer.start();
      }

      frontLiftPower = -1.0 * newAmt;

      isClimbing = newIsClimbing;
      if (isClimbing && timer.get() < 0.15) {
        frontLiftPower = Math.min(frontLiftPower, 0.30);
      } else if (isClimbing && timer.get() < 0.30) {
        frontLiftPower = Math.min(frontLiftPower, 0.15);
      }
    } else {
      frontLiftPower = -1.0 * amt * 0.33;
      isClimbing = false;
    }
    frontLiftMotor1Controller.set(frontLiftPower);
    frontLiftMotor2Controller.set(-1.0 * frontLiftPower);
  }

  /**
   * Use left operator buttons to control spider wheels:
   *   * bottom right top button moves forward
   *   * bottom right bottom button moves backwards (rarely used)
   */
  private void spiderWheels() {
    double kSpiderWheelsMaxPower = 0.5;
    if (leftOperatorStick.getRawButton(2)) {
      // spider wheels forward
      spiderWheelMotor1Controller.set(1.0*kSpiderWheelsMaxPower);
      spiderWheelMotor2Controller.set(1.0*kSpiderWheelsMaxPower);
    } else if (leftOperatorStick.getRawButton(11)) {
      // spider wheels backward
      spiderWheelMotor1Controller.set(-1.0*kSpiderWheelsMaxPower);
      spiderWheelMotor2Controller.set(-1.0*kSpiderWheelsMaxPower);
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
    double kRearLiftMaxPower = 0.5;
    if (leftOperatorStick.getRawButton(3) || rightOperatorStick.getRawButton(3)) {
      // lift
      rearLiftMotorController.set(-1.0*kRearLiftMaxPower);
    } else if (leftOperatorStick.getRawButton(4) || rightOperatorStick.getRawButton(2)) {
      // descend
      rearLiftMotorController.set(1.0*kRearLiftMaxPower);
    } else {
      // neither
      rearLiftMotorController.set(0);
    }
  }

  /**
   * Use right operator stick Y axis for controlling arm tilt.
   */
  private void armTilt() {
    double kArmTiltMaxPower = 1.0;
    double armLiftPower = kArmTiltMaxPower * smoothen(rightOperatorStick.getRawAxis(kVerticalAxis));
    armLiftPower = Math.min(armLiftPower, 0.5);
 
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
      armLongTelescopeSolenoid.set(Value.kForward);
    }else if(rightOperatorStick.getRawButton(5)){
      // retract
      armLongTelescopeSolenoid.set(Value.kReverse);
    }else{
      // do nothing
    }
  }

  /**
   * Use right operator stick trigger button for grab.
   * Operator holds button to hold ball
   */
  private void armGrab() {
    armGrabSolenoid.set(rightOperatorStick.getRawButton(1));
  }

  /**
   * Use left operator stick trigger button to punch.
   */
  private void armPunch() {
    armPunchSolenoid.set(leftOperatorStick.getRawButton(1));
  }

  /**
   * Use right operator bottom left top button to deploy arm
   */
  private void armDeploy() {
    if (rightOperatorStick.getRawButton(6) || rightOperatorStick.getRawButton(7)) {
      // deploy
      armDeploySolenoid.set(Value.kForward);
    } else {
      // undeploy
      armDeploySolenoid.set(Value.kReverse);
    }
  }

  /**
   * Not for normal use.
   * Controls arm deployment in case of failure of autonomous/end modes.
   */
  private void manualDeploy() {
    if (rightOperatorStick.getRawButton(8)) {
      armShortTelescopeSolenoid.set(true);
    } else if (rightOperatorStick.getRawButton(9)) {
      armShortTelescopeSolenoid.set(false);
    }
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

    armGrabSolenoid.set(false);
    armPunchSolenoid.set(false);

    isClimbing = false;
  }
}