package frc.AprilTags;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import frc.Robot.Constants;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.RobotPoseEstimator;
import org.photonvision.RobotPoseEstimator.PoseStrategy;

public class AprilMain extends Constants {

  public static PhotonCamera photonCamera = new PhotonCamera("robotCam");
  ;
  private static RobotPoseEstimator robotPoseEstimator;

  public AprilMain() {

    // Initialize april tags
    List<AprilTag> aprilTags = new ArrayList<>(); //vision targets

    aprilTags.add(FieldConstants.tagThree);
    aprilTags.add(FieldConstants.tagFour);
    aprilTags.add(FieldConstants.tagSix);

    // Setup AprilTags and field layout
    AprilTagFieldLayout aprilTagFieldLayout = new AprilTagFieldLayout(aprilTags,
        FieldConstants.fieldLength, FieldConstants.fieldWidth);

    var camList = new ArrayList<Pair<PhotonCamera, Transform3d>>();     //If we add multiple cameras we just add them here

    camList.add(new Pair<PhotonCamera, Transform3d>(photonCamera, VisionConstants.robotToCam));

    robotPoseEstimator = new RobotPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camList);
  }

  public static RobotPoseEstimator getRobotPoseEstimator() {
    return robotPoseEstimator;
  }

  public static Pair<Pose2d, Double> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    getRobotPoseEstimator().setReferencePose(prevEstimatedRobotPose);

    double currentTime = Timer.getFPGATimestamp();
    Optional<Pair<Pose3d, Double>> result = getRobotPoseEstimator().update(); // Most recent pose

    if (result.isPresent()) {
      return new Pair<>(
          result.get().getFirst().toPose2d(), currentTime - result.get().getSecond());
    } else {
      return new Pair<>(null, 0.0);
    }
  }
}
