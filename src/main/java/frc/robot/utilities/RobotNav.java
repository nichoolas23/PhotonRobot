package frc.robot.utilities;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Field.RoboField;
import frc.robot.subsystems.ComputerVis;
import org.photonvision.RobotPoseEstimator;

public class RobotNav {

  private static RobotPoseEstimator poseEstimator;
  private static Pose3d _robotPose3d;
  private static Pose2d _robotPose2d;
  private static AHRS _gyro = new AHRS();
  private static double _rotateVal;


  /**
   * Initializes the robot navigation data
   * Updates all poses so they aren't null when called
   */
  public static void navInit(){
    new ComputerVis();

    poseEstimator = ComputerVis.getRobotPoseEstimator();
    poseEstimator.setReferencePose(new Pose3d());
    poseEstimator.update();
    updatePose();
    _robotPose3d = poseEstimator.getReferencePose();

  }

  public static void updatePose(){
    //Updates simulated field at the same time
    _robotPose2d = RoboField.fieldUpdate(
        ComputerVis.getEstimatedGlobalPose(_robotPose3d.toPose2d()).getFirst()); // position of robot on the field
  }

  public static Pose2d getRobotPose2d() {
    return _robotPose2d;
  }

  public static AHRS getGyro() {
    return _gyro;
  }

  public static void setRotateVal(double _rotateVal) {
    RobotNav._rotateVal = _rotateVal;
  }
  }

