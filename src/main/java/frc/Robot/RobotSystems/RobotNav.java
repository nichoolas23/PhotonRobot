package frc.Robot.RobotSystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import org.photonvision.RobotPoseEstimator;

/**
 * Contains all the robots navigation data
 */
public class RobotNav {


  private static RobotPoseEstimator poseEstimator;
  private static Pose3d robotPose3d;
  private static Pose2d robotPose2d;

  private static AHRS gyro = new AHRS();
  private static double rotateVal;
}
