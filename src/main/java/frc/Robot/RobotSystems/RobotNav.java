package frc.Robot.RobotSystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.Robot.AutoDrive.AutoMode;
import java.util.Objects;
import org.photonvision.RobotPoseEstimator;

/**
 * Contains all the robots navigation data
 */
public class RobotNav {

  private RobotPoseEstimator poseEstimator;
  private Pose3d robotPose3d;
  private Pose2d robotPose2d;
  private AHRS gyro;
  private double rotateVal;

  public RobotPoseEstimator getPoseEstimator() {
    return poseEstimator;
  }

  public void setPoseEstimator(RobotPoseEstimator poseEstimator) {
    this.poseEstimator = poseEstimator;
  }

  public Pose3d getRobotPose3d() {
    return robotPose3d;
  }

  public void setRobotPose3d(Pose3d robotPose3d) {
    this.robotPose3d = robotPose3d;
  }

  public Pose2d getRobotPose2d() {
    return robotPose2d;
  }

  public void setRobotPose2d(Pose2d robotPose2d) {
    this.robotPose2d = robotPose2d;
  }

  public AHRS getGyro() {
    return gyro;
  }

  public void setGyro(AHRS gyro) {
    this.gyro = gyro;
  }

  public double getRotateVal() {
    return rotateVal;
  }

  public void setRotateVal(double rotateVal) {
    this.rotateVal = rotateVal;
  }







}
