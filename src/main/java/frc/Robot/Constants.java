package frc.Robot;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class Constants {
  //Constants setup for field at home

  public static class VisionConstants {
    public static final Transform3d robotToCam = new Transform3d(new Translation3d(0.00, .38, 0.15),
        new Rotation3d(0, 0, 0));
  }

  public static class FieldConstants {
    //APRIL TAGS
    public static AprilTag tagThree = new AprilTag(3,
        new Pose3d(new Pose2d((3.270 - .721), 4.234, Rotation2d.fromDegrees(180))));
    public static AprilTag tagFour = new AprilTag(4,
        new Pose3d(new Pose2d(0.0, 3.5189, Rotation2d.fromDegrees(270))));

    //TODO set to correct values
    public static AprilTag tagSix = new AprilTag(6,
        new Pose3d(new Pose2d(3.27, 2.66, Rotation2d.fromDegrees(90))));


    //FIELD DIMENSIONS
    public static double fieldLength = 3.27;
    public static double fieldWidth = 4.234;

  }
  static class RobotConstants{

    //robot measurements need to go here, so we can set up the robot's kinematics
  }


}
