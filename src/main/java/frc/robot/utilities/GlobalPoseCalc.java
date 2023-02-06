package frc.robot.utilities;

import static frc.robot.Constants.FieldConstants.FIELD_LENGTH;
import static frc.robot.Constants.FieldConstants.FIELD_WIDTH;
import static frc.robot.Constants.FieldConstants.TAG_FIVE;
import static frc.robot.Constants.FieldConstants.TAG_FOUR;
import static frc.robot.Constants.FieldConstants.TAG_ONE;
import static frc.robot.Constants.FieldConstants.TAG_SIX;
import static frc.robot.Constants.FieldConstants.TAG_TWO;
import static frc.robot.Constants.VisionConstants.PHOTON_CAMERA;
import static frc.robot.Constants.VisionConstants.POSE_ESTIMATOR;
import static frc.robot.Constants.VisionConstants.ROBOT_TO_CAM;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.Constants.VisionConstants;
import java.util.ArrayList;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class GlobalPoseCalc {

  public GlobalPoseCalc() {

    ArrayList<AprilTag> atList = new ArrayList<AprilTag>();
    atList.add(TAG_ONE);
    atList.add(TAG_TWO);
    atList.add(TAG_FIVE);
    atList.add(TAG_SIX);
    atList.add(TAG_FOUR);



    // TODO - once 2023 happens, replace this with just loading the 2023 field arrangement
    AprilTagFieldLayout atfl =
        new AprilTagFieldLayout(atList, FIELD_LENGTH, FIELD_WIDTH);

    // Forward Camera

    // Create pose estimator
    VisionConstants.POSE_ESTIMATOR =
        new PhotonPoseEstimator(
            atfl, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, PHOTON_CAMERA, ROBOT_TO_CAM);
    if (RobotNav.getEstimatedRobotPose() == null) {
      RobotNav.set_estimatedRobotPose(new EstimatedRobotPose(new Pose3d(), 0));
    }
    var resultPose = getEstimatedGlobalPose(
        RobotNav.getEstimatedRobotPose().estimatedPose.toPose2d());
    resultPose.ifPresent(RobotNav::set_estimatedRobotPose);

  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    POSE_ESTIMATOR.setReferencePose(prevEstimatedRobotPose);
    return POSE_ESTIMATOR.update();
  }

}
