package frc.Robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.AprilTags.AprilMain;
import frc.Field.RoboField;
import org.photonvision.RobotPoseEstimator;

/**
 * Contains all the robots navigation data
 */
public class RobotNav {

  private static RobotPoseEstimator poseEstimator;
  private static Pose3d robotPose3d;
  private static Pose2d robotPose2d;
  private static AHRS gyro;
  private static double rotateVal;


  public static void navInit(){
    poseEstimator = AprilMain.getRobotPoseEstimator();
    poseEstimator.update();
    robotPose3d = poseEstimator.getReferencePose();
  }

  public static void updatePose(){
    //Updates simulated field at the same time
    robotPose2d = RoboField.fieldUpdate(AprilMain.getEstimatedGlobalPose(robotPose3d.toPose2d()).getFirst()); // position of robot on the field
}

  public static Pose2d getRobotPose2d() {
    return robotPose2d;
  }

  public static void setRobotPose2d(Pose2d robotPose2d) {
    RobotNav.robotPose2d = robotPose2d;
  }

  public static AHRS getGyro() {
    return gyro;
  }

  public static void setGyro(AHRS gyro) {
    RobotNav.gyro = gyro;
  }

  public static double getRotateVal() {
    return rotateVal;
  }

  public static void setRotateVal(double rotateVal) {
    RobotNav.rotateVal = rotateVal;
  }
}