package frc.AprilTags;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.RobotPoseEstimator;
import org.photonvision.RobotPoseEstimator.PoseStrategy;

public class AprilSetup {

  public static PhotonCamera photonCamera = new PhotonCamera("robotCam");;
  private static RobotPoseEstimator robotPoseEstimator;

  public AprilSetup() {

    List<AprilTag> aprilTags = new ArrayList<>();

    aprilTags.add(new AprilTag(3, new Pose3d(new Pose2d((3.270 - .721), 4.234, Rotation2d.fromDegrees(180)))));
    aprilTags.add(new AprilTag(4, new Pose3d(new Pose2d(0.0, 3.5189, Rotation2d.fromDegrees(270)))));
    aprilTags.add(new AprilTag(4, new Pose3d(new Pose2d(3.27, 2.66, Rotation2d.fromDegrees(90)))));

    AprilTagFieldLayout aprilTagFieldLayout = new AprilTagFieldLayout(aprilTags, 3.27, 4.234);

    var camList = new ArrayList<Pair<PhotonCamera, Transform3d>>();

    camList.add(new Pair<PhotonCamera, Transform3d>(photonCamera, CameraConst.robotToCam));

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
