// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.Robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import frc.AprilTags.AprilMain;
import frc.Field.RoboField;
import frc.Robot.RobotSystems.Drivetrain;
import frc.Robot.RobotSystems.RobotNav;
import org.photonvision.RobotPoseEstimator;

/**
 * The VM is configured to automatically run this class, and to call the methods corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {

  private static double rotateVal;
  private static RobotPoseEstimator poseEstimator;
  private static Pose3d robotPose3d;
  private static Pose2d robotPose2d;
  Drivetrain drivetrain = new Drivetrain();
  private final Timer timer = new Timer();

  private XboxController m_controller = new XboxController(0);

  /**
   * This method is run when the robot is first started up and should be used for any initialization
   * code.
   */
  @Override
  public void robotInit() {
    RoboField.fieldSetup();
    m_controller = new XboxController(0);
    drivetrain.getRightDrive().setInverted(true);
    poseEstimator = AprilMain.getRobotPoseEstimator();
    poseEstimator.update();
    robotPose3d = poseEstimator.getReferencePose();


  }


  /**
   * This method is run once each time the robot enters autonomous mode.
   */
  @Override
  public void autonomousInit() {
    timer.reset();
    timer.start();

  }


  /**
   * This method is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    robotPose2d = RoboField.fieldUpdate(AprilMain.getEstimatedGlobalPose(robotPose3d.toPose2d())
        .getFirst()); // position of robot on the field

  }


  /**
   * This method is called once each time the robot enters teleoperated mode.
   */
  @Override
  public void teleopInit() {
    drivetrain.getLeftDrive().set(0);
    drivetrain.getRightDrive().set(0);
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void robotPeriodic() {

  }

  /**
   * This method is called periodically during teleoperated mode.
   */
  @Override
  public void teleopPeriodic() {
    if (m_controller.getAButtonPressed()) {
      drivetrain.getLeftDrive().stopMotor();
      drivetrain.getRightDrive().stopMotor();
    }

    double forward = m_controller.getLeftTriggerAxis() * .6;
    double reverse = m_controller.getRightTriggerAxis() * .6;
    rotateVal = m_controller.getRightX() * .6;

    drivetrain.getLeftDrive().set(forward + rotateVal);
    drivetrain.getRightDrive().set(forward - rotateVal);

    drivetrain.getLeftDrive().set(-1.0 * reverse - rotateVal);
    drivetrain.getRightDrive().set(-1.0 * reverse + rotateVal);


  }


  //TODO: Setup photonvision sim testing
  @Override
  public void testInit() {

  }

  @Override
  public void simulationInit() {

  }

  @Override
  public void simulationPeriodic() {

  }

  @Override
  public void testPeriodic() {
  }
}
