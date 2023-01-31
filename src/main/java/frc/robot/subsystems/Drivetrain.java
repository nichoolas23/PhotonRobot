package frc.robot.subsystems;

import static frc.robot.Constants.FieldConstants.TAG_THREE;
import static frc.robot.Constants.RobotConstants.TRACK_WIDTH;
import static frc.robot.Constants.VisionConstants.PHOTON_CAMERA;
import static frc.robot.Constants.VisionConstants.ROBOT_TO_CAM;
import static frc.robot.utilities.RobotNav.getGyro;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.controller.DifferentialDriveWheelVoltages;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;
import frc.robot.utilities.RobotNav;

public class Drivetrain extends SubsystemBase {


// Start motor setup
  private final WPI_TalonSRX[] wpi_talonSRXES = new WPI_TalonSRX[]{new WPI_TalonSRX(1),
      new WPI_TalonSRX(3), new WPI_TalonSRX(2), new WPI_TalonSRX(4)};
  private final MotorControllerGroup _leftDrive = new MotorControllerGroup(wpi_talonSRXES[0], wpi_talonSRXES[1]);
  private final MotorControllerGroup _rightDrive = new MotorControllerGroup(wpi_talonSRXES[2], wpi_talonSRXES[3]);

  // End motor setup

  private Encoder _leftEncoder = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
   private Encoder _rightEncoder = new Encoder(2, 3, true, EncodingType.k1X);

  //  Start DifferentialDrive setup
  private final DifferentialDrive _differentialDrive = new DifferentialDrive(_leftDrive,
      _rightDrive);
  private DifferentialDriveWheelSpeeds _driveWheelSpeeds = new DifferentialDriveWheelSpeeds(0, 0);
  private DifferentialDriveWheelVoltages _driveWheelVoltages;
  private DifferentialDriveOdometry _driveOdometry = new DifferentialDriveOdometry(getGyro().getRotation2d(),_leftEncoder.getDistance(),_rightEncoder.getDistance());
  private DifferentialDriveKinematics _driveKinematics = new DifferentialDriveKinematics(TRACK_WIDTH);
// DifferentialDriveKinematics kinematics,
//      Rotation2d gyroAngle,
//      double leftDistanceMeters,
//      double rightDistanceMeters,
//      Pose2d initialPoseMeters) {
  DifferentialDrivePoseEstimator _poseEstimator = new DifferentialDrivePoseEstimator(_driveKinematics, getGyro().getRotation2d(), _leftEncoder.getDistance(), _rightEncoder.getDistance(),
    RobotNav.getRobotPose2d());
  public Drivetrain() {
    _leftEncoder.setDistancePerPulse(RobotConstants.DISTANCE_PER_PULSE);
    _rightDrive.setInverted(true);
    _differentialDrive.setSafetyEnabled(false);
  }

  @Override
  public void periodic() {
  updateOdometry();
  }

  public void updateOdometry() {
    var gyroAngle = getGyro().getRotation2d();
    _driveWheelSpeeds = new DifferentialDriveWheelSpeeds(_leftEncoder.getRate(), _rightEncoder.getRate());

    _driveWheelVoltages.left = wpi_talonSRXES[1].getMotorOutputVoltage()+wpi_talonSRXES[0].getMotorOutputVoltage();
    _driveWheelVoltages.right = wpi_talonSRXES[2].getMotorOutputVoltage()+wpi_talonSRXES[3].getMotorOutputVoltage();
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    _driveOdometry.resetPosition(
        getGyro().getRotation2d(), _leftEncoder.getDistance(), _rightEncoder.getDistance(), pose);
  }

  /**
   * Feeds in vision data to the pose estimator.
   * @param leftDist
   * @param rightDist
   */
  public void cvCorrectPose(double leftDist, double rightDist) {
    _poseEstimator.update(getGyro().getRotation2d(), leftDist, rightDist);

    var res = PHOTON_CAMERA.getLatestResult();
    if (res.hasTargets()) {
      var imageCaptureTime = res.getTimestampSeconds();

      var camToTargetTrans = res.getBestTarget().getBestCameraToTarget();
      var camPose = TAG_THREE.pose.transformBy(camToTargetTrans.inverse());
      _poseEstimator.addVisionMeasurement(
          camPose.transformBy(ROBOT_TO_CAM).toPose2d(), imageCaptureTime);
    }
  }

  private void resetEncoders() {
    _leftEncoder.reset();
    _rightEncoder.reset();
  }

  public void setDriveVolts(double leftVolts, double rightVolts) {
    _leftDrive.setVoltage(leftVolts);
    _rightDrive.setVoltage(rightVolts);
    _differentialDrive.feed();

  }

/**
* Controls the driving mechanism of the robot.
 * @param forwardSpeed Between 0 and 1.0
 * @param reverseSpeed Between 0 and 1.0 inverted for driving backwards
 * @param rot Between -1.0 and 1.0 for turning
*/
  public void drive(double forwardSpeed, double reverseSpeed, double rot, boolean isAuto ) {

    _differentialDrive.arcadeDrive(reverseSpeed > 0 ? reverseSpeed*-1 : forwardSpeed, rot);

  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(_leftEncoder.getRate(), _rightEncoder.getRate());
  }

}
