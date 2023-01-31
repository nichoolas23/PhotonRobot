package frc.robot.subsystems;

import static frc.robot.Constants.VisionConstants.PHOTON_CAMERA;


import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.RobotPoseEstimator;
import org.photonvision.RobotPoseEstimator.PoseStrategy;

// AI generated tldr lmao
/*
1. We create a list of AprilTags that we want to use.
2. We create an AprilTagFieldLayout with the list of tags and the field dimensions.
3. We create a list of cameras, which in our case is just one camera,
and add it to the list along with its transform from robot space to camera space
(which we defined in `VisionConstants`).
4. We initialize a RobotPoseEstimator object with the field layout,
pose strategy (we chose closest to reference pose), and camera list from above.
5. The getEstimatedGlobalPose method takes in our previous estimated robot pose as an argument
and returns our current estimated robot pose along with how long ago it was calculated
(in seconds).This is useful for knowing if you are using stale data or not when you are trying to drive towards something based on vision data!
 */

public class ComputerVis extends SubsystemBase {


  private static RobotPoseEstimator robotPoseEstimator;

  public ComputerVis() {

    // Initialize april tags
    List<AprilTag> aprilTags = new ArrayList<>(); //vision targets

    aprilTags.add(FieldConstants.TAG_THREE);
    aprilTags.add(FieldConstants.TAG_FOUR);
    aprilTags.add(FieldConstants.TAG_SIX);


    // Setup AprilTags and field layout
    AprilTagFieldLayout aprilTagFieldLayout = new AprilTagFieldLayout(aprilTags,
        FieldConstants.FIELD_LENGTH, FieldConstants.FIELD_WIDTH);

    var camList = new ArrayList<Pair<PhotonCamera, Transform3d>>();    //If we add multiple cameras we just add them here

    camList.add(new Pair<PhotonCamera, Transform3d>(PHOTON_CAMERA, VisionConstants.ROBOT_TO_CAM));

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
