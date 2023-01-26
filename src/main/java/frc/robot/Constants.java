package frc.robot;

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
    public static final Transform3d ROBOT_TO_CAM = new Transform3d(new Translation3d(0.00, .38, 0.15),
        new Rotation3d(0, 0, 0));
  }

  public static class FieldConstants  {
    /**
     *April Tags
     */
    public static AprilTag TAG_THREE = new AprilTag(3, new Pose3d(new Pose2d((3.270 - .721), 4.234, Rotation2d.fromDegrees(180))));
    public static AprilTag TAG_FOUR = new AprilTag(4,
        new Pose3d(new Pose2d(0.0, 3.5189, Rotation2d.fromDegrees(270))));

    //TODO set to correct values
    public static AprilTag TAG_SIX = new AprilTag(6,
        new Pose3d(new Pose2d(3.27, 2.66, Rotation2d.fromDegrees(90))));

    public static Pose2d FIRST_BLUE_GRID = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));
    public static Pose2d SECOND_BLUE_GRID = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));
    public static Pose2d THIRD_BLUE_GRID = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));
    public static Pose2d FOURTH_BLUE_GRID = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));
    public static Pose2d FIFTH_BLUE_GRID = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));

    public static Pose2d BLUE_CHARGING_STATION = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));
    public static Pose2d RED_CHARGING_STATION = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));

    public static Pose2d FIRST_RED_GRID = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));
    public static Pose2d SECOND_RED_GRID = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));
    public static Pose2d THIRD_RED_GRID = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));
    public static Pose2d FOURTH_RED_GRID = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));
    public static Pose2d FIFTH_RED_GRID = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));


       /**
     *  Field Dimensions
     */
    public static double FIELD_LENGTH = 3.27;
    public static double FIELD_WIDTH = 4.234;


  }
  static class RobotConstants {

    /**
     *Forward PID
     */
    final double LINEAR_P = 0.1;
    final double LINEAR_D = 0.0;
    /**
     *Turning PID
     */
    final double ANGULAR_P = 0.1;
    final double ANGULAR_D = 0.0;
    //robot measurements need to go here, so we can set up the robot's kinematics
  }


}
