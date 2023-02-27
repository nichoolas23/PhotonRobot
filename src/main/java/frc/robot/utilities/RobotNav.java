package frc.robot.utilities;

import static frc.robot.Constants.RobotConstants.ENCODER_SCALE_CONSTANT;
import static frc.robot.Constants.VisionConstants.VISION_STD_DEV;

import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.utilities.LimelightHelpers.LimelightResults;
import java.awt.Point;
import java.awt.geom.Point2D;


public class RobotNav {

  private static DifferentialDrivePoseEstimator _diffDrivePose;
  private static final AHRS _gyro = new AHRS();
  private static double _rotateVal;

  private static SensorCollection _leftEncoder = new WPI_TalonSRX(0).getSensorCollection();
  private static SensorCollection _rightEncoder = new WPI_TalonSRX(5).getSensorCollection();
  //Limelight data


  public static void setStdDevVision() {
    VISION_STD_DEV.set(0, 0, 0.5);
    VISION_STD_DEV.set(1, 0, 0.5);
    VISION_STD_DEV.set(2, 0, Math.toRadians(30));
  }


  /**
   * Initializes the robot navigation data Updates all poses so they aren't null when called
   */
  public RobotNav() {

  }

  public static AHRS getGyro() {
    return _gyro;
  }

  public static void setRotateVal(double _rotateVal) {
    RobotNav._rotateVal = _rotateVal;
  }

  public static Pose2d getFieldAdjPose(Pose2d pose2d) {
    var megaBotPose = LimelightHelpers.getBotPose2d("");
    return new Pose2d(new Translation2d(pose2d.getX() + 2.56, pose2d.getY() + 1.8825),
        pose2d.getRotation());

  }

  public static DifferentialDrivePoseEstimator get_diffDrivePose() {
    return _diffDrivePose;
  }

  public static void set_diffDrivePose(
      DifferentialDrivePoseEstimator _diffDrivePose) {
    RobotNav._diffDrivePose = _diffDrivePose;
  }
  public static Pose2d getEstPose() {
    return _diffDrivePose.getEstimatedPosition();
  }

  public static SensorCollection get_rightEncoder() {
    return _rightEncoder;
  }

  public static SensorCollection get_leftEncoder() {
    return _leftEncoder;
  }


  public void updateLL() {
      LimelightHelpers.getLatestResults("limelight");
  }

  public static double getHeading() {
    return _gyro.getFusedHeading();
  }
  public double getTurnRate() {return _gyro.getRate();}
  public double getRobotPitch() {
    return _gyro.getPitch();}

  public static double getLeftEncoderPosition() {
    return _leftEncoder.getQuadraturePosition() *ENCODER_SCALE_CONSTANT;
  }
  public static double getRightEncoderPosition() {
    return _rightEncoder.getQuadraturePosition() *ENCODER_SCALE_CONSTANT;
  }
  public static double getLeftEncoderVelocity() {
    return _leftEncoder.getQuadratureVelocity() *ENCODER_SCALE_CONSTANT;
  }
  public static double getRightEncoderVelocity() {
    return _rightEncoder.getQuadratureVelocity() *ENCODER_SCALE_CONSTANT;
  }
}

