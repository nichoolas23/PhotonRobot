package frc.robot.utilities;

import static frc.robot.Constants.FieldConstants.FIELD_LENGTH;
import static frc.robot.Constants.FieldConstants.FIELD_WIDTH;
import static frc.robot.Constants.FieldConstants.TAG_FOUR;
import static frc.robot.Constants.FieldConstants.TAG_THREE;
import static frc.robot.Constants.VisionConstants.PHOTON_CAMERA;
import static frc.robot.Constants.VisionConstants.POSE_ESTIMATOR;
import static frc.robot.Constants.VisionConstants.ROBOT_TO_CAM;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import frc.robot.Constants.VisionConstants;
import java.util.ArrayList;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class GlobalPoseCalc {

 public GlobalPoseCalc(){

   ArrayList<AprilTag> atList = new ArrayList<AprilTag>();
   atList.add(TAG_THREE);
   atList.add(TAG_FOUR);

   // TODO - once 2023 happens, replace this with just loading the 2023 field arrangement
   AprilTagFieldLayout atfl =
       new AprilTagFieldLayout(atList, FIELD_LENGTH, FIELD_WIDTH);

   // Forward Camera



   // Create pose estimator
   VisionConstants.POSE_ESTIMATOR =
       new PhotonPoseEstimator(
           atfl, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, PHOTON_CAMERA, ROBOT_TO_CAM);
if(RobotNav.get_estimatedRobotPose().estimatedPose == null){
  RobotNav.set_estimatedRobotPose(new EstimatedRobotPose(new Pose3d(), 0));
}
   RobotNav.set_estimatedRobotPose(getEstimatedGlobalPose(RobotNav.get_estimatedRobotPose().estimatedPose.toPose2d()).get());
 }
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    POSE_ESTIMATOR.setReferencePose(prevEstimatedRobotPose);
    return POSE_ESTIMATOR.update();
  }

}
