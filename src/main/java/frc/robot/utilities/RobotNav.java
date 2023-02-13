package frc.robot.utilities;

import static frc.robot.Constants.VisionConstants.VISION_STD_DEV;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;


public class RobotNav {

  private static AHRS _gyro = new AHRS();
  private static double _rotateVal;

  //Limelight data


  public static void setStdDevVision() {
    VISION_STD_DEV.set(0, 0, 0.01);
    VISION_STD_DEV.set(1, 0, 0.01);
    VISION_STD_DEV.set(2, 0, Math.toRadians(2));
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

  public void updateLL() {
    LimelightHelpers.getLatestResults("limelight");

  }
}

