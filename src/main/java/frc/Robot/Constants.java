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

  public static class FieldConstants  {
    /**
     *April Tags
     */
    public static AprilTag tagThree = new AprilTag(3, new Pose3d(new Pose2d((3.270 - .721), 4.234, Rotation2d.fromDegrees(180))));
    public static AprilTag tagFour = new AprilTag(4,
        new Pose3d(new Pose2d(0.0, 3.5189, Rotation2d.fromDegrees(270))));

    //TODO set to correct values
    public static AprilTag tagSix = new AprilTag(6,
        new Pose3d(new Pose2d(3.27, 2.66, Rotation2d.fromDegrees(90))));

    public static Pose2d firstBlueGrid = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));
    public static Pose2d secondBlueGrid = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));
    public static Pose2d thirdBlueGrid = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));
    public static Pose2d fourthBlueGrid = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));
    public static Pose2d fifthBlueGrid = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));

    public static Pose2d blueChargingStation = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));
    public static Pose2d redChargingStation = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));

    public static Pose2d firstRedGrid = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));
    public static Pose2d secondRedGrid = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));
    public static Pose2d thirdRedGrid = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));
    public static Pose2d fourthRedGrid = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));
    public static Pose2d fifthRedGrid = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));


       /**
     *  Field Dimensions
     */
    public static double fieldLength = 3.27;
    public static double fieldWidth = 4.234;


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
