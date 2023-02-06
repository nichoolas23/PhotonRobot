package frc.robot.utilities;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import org.photonvision.EstimatedRobotPose;

public class RobotNav {

  private static AHRS _gyro = new AHRS();
  private static double _rotateVal;
  private static EstimatedRobotPose _estimatedRobotPose;

  /**
   * Initializes the robot navigation data
   * Updates all poses so they aren't null when called
   */
  public static void navInit(){

  }

  public static AHRS getGyro() {
    return _gyro;
  }

  public static void setRotateVal(double _rotateVal) {
    RobotNav._rotateVal = _rotateVal;
  }

  public static EstimatedRobotPose getEstimatedRobotPose() {
    return _estimatedRobotPose;
  }
  public Pose2d getEstimatedRobotPose2d() {
    return _estimatedRobotPose.estimatedPose.toPose2d();
  }

  public static void set_estimatedRobotPose(EstimatedRobotPose _estimatedRobotPose) {
    RobotNav._estimatedRobotPose = _estimatedRobotPose;
  }
}

