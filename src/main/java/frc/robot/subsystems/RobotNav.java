package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Field.RoboField;
import org.photonvision.RobotPoseEstimator;

public class RobotNav extends SubsystemBase {

  private static RobotPoseEstimator poseEstimator;
  private static Pose3d _robotPose3d;
  private static Pose2d _robotPose2d;
  private static AHRS _gyro = new AHRS();
  private static double _rotateVal;


  public static void navInit(){
    new AprilMain();

    poseEstimator = AprilMain.getRobotPoseEstimator();
    poseEstimator.setReferencePose(new Pose3d());

    poseEstimator.update();

    _robotPose3d = poseEstimator.getReferencePose();

  }

  public static void updatePose(){
    //Updates simulated field at the same time
    _robotPose2d = RoboField.fieldUpdate(AprilMain.getEstimatedGlobalPose(_robotPose3d.toPose2d()).getFirst()); // position of robot on the field
  }

  public static Pose2d get_robotPose2d() {
    return _robotPose2d;
  }

  public static void set_robotPose2d(Pose2d _robotPose2d) {
    RobotNav._robotPose2d = _robotPose2d;
  }

  public static AHRS get_gyro() {
    return _gyro;
  }

  public static void set_gyro(AHRS _gyro) {
    RobotNav._gyro = _gyro;
  }

  public static double get_rotateVal() {
    return _rotateVal;
  }

  public static void set_rotateVal(double _rotateVal) {
    RobotNav._rotateVal = _rotateVal;
  }
  }

