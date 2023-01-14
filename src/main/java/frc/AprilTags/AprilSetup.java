package frc.AprilTags;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import frc.Field.RoboField;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.RobotPoseEstimator;
import org.photonvision.RobotPoseEstimator.PoseStrategy;

public class AprilSetup extends RobotConstants {

  public static PhotonCamera photonCamera = new PhotonCamera("robotCam");;
  private static RobotPoseEstimator robotPoseEstimator;

  public AprilSetup() {

    List<AprilTag> aprilTags = new ArrayList<>();

    aprilTags.add(FieldConstants.tagThree);
    aprilTags.add(FieldConstants.tagFour);
    aprilTags.add(FieldConstants.tagSix);

    AprilTagFieldLayout aprilTagFieldLayout = new AprilTagFieldLayout(aprilTags, 3.27, 4.234);

    var camList = new ArrayList<Pair<PhotonCamera, Transform3d>>();

    camList.add(new Pair<PhotonCamera, Transform3d>(photonCamera, VisionConstants.robotToCam));

    robotPoseEstimator = new RobotPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camList);
  }

  public static RobotPoseEstimator getRobotPoseEstimator() {
    return robotPoseEstimator;
  }


  public static Pair<Pose2d, Double> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    getRobotPoseEstimator().setReferencePose(prevEstimatedRobotPose);

    double currentTime = Timer.getFPGATimestamp();
    Optional<Pair<Pose3d, Double>> result = getRobotPoseEstimator().update();
    if (result.isPresent()) {
      return new Pair<Pose2d, Double>(
          result.get().getFirst().toPose2d(), currentTime - result.get().getSecond());
    } else {
      return new Pair<Pose2d, Double>(null, 0.0);
    }
  }
}
