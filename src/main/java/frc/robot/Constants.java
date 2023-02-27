package frc.robot;

import static frc.robot.Constants.RobotConstants.PhysicalConstants.TRACK_WIDTH;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.subsystems.RobotAlignment;
import frc.robot.utilities.RobotNav;
import java.nio.charset.StandardCharsets;


/**
* The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean values.
 * This class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do not put anything functional in this class.
*/
public class Constants {


  public static class VisionConstants {

    public static final Transform3d ROBOT_TO_CAM = new Transform3d(new Translation3d(0.00, .38, 0.15),
        new Rotation3d(0, 0, 0));
    public static Matrix<N3, N1> VISION_STD_DEV = new Matrix<>(N3.instance, N1.instance);


  }

  public static class FieldConstants  {


    /**
     *April Tags
     */
    public static AprilTag TAG_ONE = new AprilTag(1, new Pose3d(new Pose2d(2.607, 4.392, Rotation2d.fromDegrees(270))));//00

    public static AprilTag TAG_TWO = new AprilTag(2, new Pose3d(new Pose2d(1.69, 4.392, Rotation2d.fromDegrees(270))));

    public static AprilTag TAG_FOUR = new AprilTag(4, new Pose3d(new Pose2d(5.72, 2.357, Rotation2d.fromDegrees (180))));

    public static AprilTag TAG_FIVE = new AprilTag(5, new Pose3d(new Pose2d(0, 2.164, Rotation2d.fromDegrees(0))));

    public static AprilTag TAG_SIX = new AprilTag(6, new Pose3d(new Pose2d(5.720, 3.275, Rotation2d.fromDegrees(180))));


    public static Pose2d FIRST_BLUE_GRID = new Pose2d( 2.56+2.2, 0.3065+ 1.8825, Rotation2d.fromDegrees(0));
    public static Pose2d SECOND_BLUE_GRID = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));
    public static Pose2d THIRD_BLUE_GRID = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));
    public static Pose2d FOURTH_BLUE_GRID = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));
      public static Pose2d FIFTH_BLUE_GRID = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));

    public static Pose2d BLUE_CHARGING_STATION = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));
    public static Pose2d RED_CHARGING_STATION = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));

    public static Pose2d FIRST_RED_GRID = new Pose2d(2.607, 4.392, Rotation2d.fromDegrees(270));

    public static Pose2d LOLA = new Pose2d("LolaX".getBytes(StandardCharsets.UTF_8).length,
        "LolaY".getBytes(StandardCharsets.UTF_8).length ,
        Rotation2d.fromDegrees(270));
    public static Pose2d SECOND_RED_GRID = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));
    public static Pose2d THIRD_RED_GRID = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));
    public static Pose2d FOURTH_RED_GRID = new Pose2d(5.72, 2.357, Rotation2d.fromDegrees (180));
    public static Pose2d FIFTH_RED_GRID = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));


       /**
     *  Field Dimensions
     */
    public static double FIELD_LENGTH = 3.27;
    public static double FIELD_WIDTH = 4.234;


  }
  public static class RobotConstants {



   public static class PhysicalConstants {
     public static final double WHEEL_CIRCUM = .6283185;
     public static final double TRACK_WIDTH = 0.72;
   }
    public static double ENCODER_SCALE_CONSTANT =  ((Units.inchesToMeters(6)*Math.PI)/ 1085);
    // Max speed of the robot in m/s
    public static DifferentialDriveKinematics DRIVE_KINEMATICS =
        new DifferentialDriveKinematics(TRACK_WIDTH);
    public static final double TURN_DEG_PER_SEC_MAX = 10;
    public static final double TURN_ACCEL_DEG_PER_SECSQ_MAX = 2;


    public static final double RAMSETE_B = 2; // Tuning parameter (b > 0 rad^2/m^2) for which larger values make convergence more aggressive like a proportional term.
    public static final double RAMSETE_ZETA = 0.5; // Tuning parameter (0 rad-1 < zeta < 1 rad-1) for which larger values provide more damping in response.


//RAMSETE SETUP
    public static double AUTO_MAX_SPEED = 3;
    public static double AUTO_MAX_ACCEL = 3;

    public static double P_GAIN_DRIVE_VEL = 3.1519;
    public static double VOLTS_MAX = 1.103;
    public static double VOLTS_SECONDS_PER_METER = 2.0061;
    public static double VOLTS_SECONDS_SQ_PER_METER = 1.4236;

    /*public static RectangularRegionConstraint FIELD_CONSTRAINT =
        new RectangularRegionConstraint(1.0,2.0, 1.0, 2.00);*/

    public static DifferentialDriveVoltageConstraint AUTO_VOLTAGE_CONSTRAINT =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                VOLTS_MAX,
                VOLTS_SECONDS_PER_METER,
                VOLTS_SECONDS_SQ_PER_METER),
            DRIVE_KINEMATICS,
            10);

    public static TrajectoryConfig TRAJ_CONFIG =
        new TrajectoryConfig(
            AUTO_MAX_SPEED,
            AUTO_MAX_ACCEL)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DRIVE_KINEMATICS)
            // Apply the voltage constraint
            .addConstraint(AUTO_VOLTAGE_CONSTRAINT);


    public static class PneumaticsConstants {
      public static Solenoid WRIST_PISTON = new Solenoid(PneumaticsModuleType.CTREPCM, 12);

      public record RPistonControl(Solenoid solenoid, DoubleSolenoid doubleSolenoid, double pulseDuration, long delay) {}


    }

    public static class ControlsConstants{

      public static RobotAlignment ALIGNMENT = new RobotAlignment(new ProfiledPIDController(0.0, 0.0, 0.0,new TrapezoidProfile.Constraints(
          TURN_DEG_PER_SEC_MAX,
          TURN_ACCEL_DEG_PER_SECSQ_MAX)),
          RobotNav.getGyro().getFusedHeading());
    }
    public static double STAB_PID_P = 0;
    public static double STAB_PID_I = 0;
    public static double STAB_PID_D = 0.0;

  }


}
