package frc.robot.subsystems;

import static frc.robot.Constants.RobotConstants.DRIVE_KINEMATICS;
import static frc.robot.Constants.RobotConstants.LEFT_ENCODER;
import static frc.robot.Constants.RobotConstants.PhysicalConstants.WHEEL_CIRCUM;
import static frc.robot.Constants.RobotConstants.RIGHT_ENCODER;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.controller.DifferentialDriveWheelVoltages;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {

/*
private final WPI_TalonSRX[] wpi_talonSRXES = new WPI_TalonSRX[]{new WPI_TalonSRX(1),
      new WPI_TalonSRX(3), new WPI_TalonSRX(2), new WPI_TalonSRX(4)};
  private final MotorControllerGroup _leftDrive = new MotorControllerGroup(wpi_talonSRXES[0],
      wpi_talonSRXES[1]);
  private final MotorControllerGroup _rightDrive = new MotorControllerGroup(wpi_talonSRXES[2],
      wpi_talonSRXES[3]);
 */
  private final WPI_TalonSRX[] wpi_talonSRXES = new WPI_TalonSRX[]{new WPI_TalonSRX(1),
      new WPI_TalonSRX(3), new WPI_TalonSRX(2), new WPI_TalonSRX(4)};
  private final MotorControllerGroup _leftDrive = new MotorControllerGroup(wpi_talonSRXES[1],
      wpi_talonSRXES[3]);
  private final MotorControllerGroup _rightDrive = new MotorControllerGroup(wpi_talonSRXES[0],
      wpi_talonSRXES[2]);
  private final AHRS _gyro = new AHRS();
  private final DifferentialDrive _differentialDrive = new DifferentialDrive(_leftDrive,
      _rightDrive);

  // Start motor setup

  private DifferentialDrivePoseEstimator _diffPoseEstimator =
      new DifferentialDrivePoseEstimator(
          DRIVE_KINEMATICS, _gyro.getRotation2d(), 0.0, 0.0, new Pose2d());
  private DifferentialDriveWheelSpeeds _diffDriveWheelSpeeds = new DifferentialDriveWheelSpeeds();
  private DifferentialDriveWheelVoltages _diffDriveWheelVoltages = new DifferentialDriveWheelVoltages();
  private DifferentialDriveOdometry _diffDriveOdometry = new DifferentialDriveOdometry(_gyro.getRotation2d(),0,0);



  public Drivetrain() {
    LEFT_ENCODER.setDistancePerPulse(WHEEL_CIRCUM/ 357.75);
    LEFT_ENCODER.setReverseDirection(true);
    RIGHT_ENCODER.setDistancePerPulse(WHEEL_CIRCUM / 355);

    _rightDrive.setInverted(true);
    _differentialDrive.setSafetyEnabled(false);

    _diffPoseEstimator.update(_gyro.getRotation2d(), 0, 0);
  }


  @Override
  public void periodic() {


 /*   if(isNearLola()){
      Trajectory nickTrajectory = TrajectoryGenerator.generateTrajectory(
          RobotNav.getEstimatedRobotPose().estimatedPose.toPose2d(), List.of(),
          FieldConstants.NICK,
          new TrajectoryConfig(2, 2)
      );
      Command escapeLola = new PathFollowCmd(nickTrajectory);
      escapeLola.schedule();
    }
  }
  public boolean isNearLola(){
    Point position = new Point((int) RobotNav.getEstimatedRobotPose().estimatedPose.getX(),
        (int) RobotNav.getEstimatedRobotPose().estimatedPose.getY());

    return position.distance(lola) < 10;
  }*/
  }


  public void updateOdometry() {

    _diffDriveOdometry.update(_gyro.getRotation2d(), LEFT_ENCODER.getDistance(), RIGHT_ENCODER.getDistance());

    _diffDriveWheelSpeeds.leftMetersPerSecond = LEFT_ENCODER.getRate();
    _diffDriveWheelSpeeds.rightMetersPerSecond = RIGHT_ENCODER.getRate();

    _diffDriveWheelVoltages.left = wpi_talonSRXES[0].getMotorOutputVoltage() + wpi_talonSRXES[1].getMotorOutputVoltage();
    _diffDriveWheelVoltages.right = wpi_talonSRXES[3].getMotorOutputVoltage() + wpi_talonSRXES[2].getMotorOutputVoltage();

    _diffPoseEstimator.update(
        _gyro.getRotation2d(), LEFT_ENCODER.getDistance(), RIGHT_ENCODER.getDistance());
    SmartDashboard.putNumber("Left Encoder", LEFT_ENCODER.getDistance());
    SmartDashboard.putNumber("Right Encoder", RIGHT_ENCODER.getDistance());
    // Also apply vision measurements. We use 0.3 seconds in the past as an example
    // -- on
    // a real robot, this must be calculated based either on latency or timestamps.

  }


  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return _diffDriveWheelSpeeds;
  }
  public void setVoltages(double leftVolts, double rightVolts) {
    _leftDrive.setVoltage(leftVolts*.25);
    _rightDrive.setVoltage(rightVolts*.25);
  }
  public void resetOdometry(Pose2d initialPose) {
    resetEncoders();
    _diffDriveOdometry.resetPosition(
        _gyro.getRotation2d(), LEFT_ENCODER.getDistance(), RIGHT_ENCODER.getDistance(), initialPose);
  }

  public void resetEncoders() {
    LEFT_ENCODER.reset();
    RIGHT_ENCODER.reset();
  }




  /**
   * Controls the driving mechanism of the robot.
   *
   * @param forwardSpeed Between 0 and 1.0
   * @param reverseSpeed Between 0 and 1.0 inverted for driving backwards
   * @param rot          Between -1.0 and 1.0 for turning
   */
  public void drive(double forwardSpeed, double reverseSpeed, double rot, boolean isAuto,
      boolean isTracking) {

    _differentialDrive.arcadeDrive(reverseSpeed > 0 ? reverseSpeed * -1 : forwardSpeed, rot * -1);
    RamseteController ramseteController = new RamseteController();
  }



}
