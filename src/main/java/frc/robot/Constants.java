package frc.robot;

import static frc.robot.Constants.RobotConstants.PhysicalConstants.TRACK_WIDTH;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
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

    public static final Transform3d ROBOT_TO_CAM = new Transform3d(
        new Translation3d(0.00, .38, 0.15),
        new Rotation3d(0, 0, 0));
    public static Matrix<N3, N1> VISION_STD_DEV = new Matrix<>(N3.instance, N1.instance);


  }

  public static class FieldConstants {


    /**
     * April Tags
     */
    public static AprilTag RED_GRID_BOTTOM_LEFT = new AprilTag(1,
        new Pose3d(new Pose2d(7.24310, -2.93659, Rotation2d.fromDegrees(270))));//00

    public static AprilTag RED_GRID_MIDDLE_RIGHT = new AprilTag(2,
        new Pose3d(new Pose2d(7.24310, -1.26019, Rotation2d.fromDegrees(270))));

    public static AprilTag RED_GRID_TOP_RIGHT = new AprilTag(3,
        new Pose3d(new Pose2d(7.24310, 0.41621, Rotation2d.fromDegrees(180))));

    public static AprilTag BLUE_PICKUP = new AprilTag(4,
        new Pose3d(new Pose2d(7.90832, 2.74161, Rotation2d.fromDegrees(180))));

    public static AprilTag RED_PICKUP = new AprilTag(5,
        new Pose3d(new Pose2d(-7.90832, 2.74161, Rotation2d.fromDegrees(0))));

    public static AprilTag BLUE_GRID_TOP_LEFT = new AprilTag(6,
        new Pose3d(new Pose2d(-7.24310, 0.41621, Rotation2d.fromDegrees(180))));
    public static AprilTag BLUE_GRID_MIDDLE_LEFT = new AprilTag(7,
        new Pose3d(new Pose2d(-7.24310, -1.26019, Rotation2d.fromDegrees(270))));
    public static AprilTag BLUE_GRID_BOTTOM_LEFT = new AprilTag(8,
        new Pose3d(new Pose2d(-7.24310, -2.93659, Rotation2d.fromDegrees(270))));


    public static Pose2d LOLA = new Pose2d("LolaX".getBytes(StandardCharsets.UTF_8).length,
        "LolaY".getBytes(StandardCharsets.UTF_8).length,
        Rotation2d.fromDegrees(270));
    public static Pose2d SECOND_RED_GRID = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));
    public static Pose2d THIRD_RED_GRID = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));
    public static Pose2d FOURTH_RED_GRID = new Pose2d(5.72, 2.357, Rotation2d.fromDegrees(180));
    public static Pose2d FIFTH_RED_GRID = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));


    /**
     * Field Dimensions
     */
    public static double FIELD_LENGTH = 3.27;
    public static double FIELD_WIDTH = 4.234;


  }

  public static class RobotConstants {


    public static class PhysicalConstants {

      public static final double WHEEL_CIRCUM = .6283185;
      public static final double TRACK_WIDTH = 0.72;
    }

    public static double ENCODER_SCALE_CONSTANT = ((Units.inchesToMeters(6) * Math.PI) / 1024);
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

    //COMBINED FEED FORWARD
    public static double COMBINED_VOLTS_MAX = 0.65488;
    public static double COMBINED_VOLTS_SECONDS_PER_METER = 2.7164;
    public static double COMBINED_VOLTS_SECONDS_SQ_PER_METER = 0.10304;
    //LEFT SIDE FEED FORWARD
    public static double LEFT_VOLTS_MAX = 0.91107;
    public static double LEFT_VOLTS_SECONDS_PER_METER = 1.0051;
    public static double LEFT_VOLTS_SECONDS_SQ_PER_METER = 0.58786;

    // LEFT SIDE FEEDBACK
    public static double LEFT_VEL_METERS_PER_SECOND = 3.4268;

    //RIGHT SIDE FEED FORWARD
    public static double RIGHT_VOLTS_MAX = 0.74523;
    public static double RIGHT_VOLTS_SECONDS_PER_METER = 2.5707;
    public static double RIGHT_VOLTS_SECONDS_SQ_PER_METER = 0.37322;
    /*public static RectangularRegionConstraint FIELD_CONSTRAINT =
        new RectangularRegionConstraint(1.0,2.0, 1.0, 2.00);*/

    // POSITION PID CONSTANTS
    public static double COMBINED_P_GAIN_POS = 0.19333;
    public static double COMBINED_D_GAIN_POS = 0.022148;
    public static DifferentialDriveVoltageConstraint AUTO_VOLTAGE_CONSTRAINT =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                COMBINED_VOLTS_MAX,
                COMBINED_VOLTS_SECONDS_PER_METER,
                COMBINED_VOLTS_SECONDS_SQ_PER_METER),
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


      public static Solenoid LEFT_HILO_GEARSHIFT = new Solenoid(PneumaticsModuleType.CTREPCM, 3);
      public static Solenoid RIGHT_HILO_GEARSHIFT = new Solenoid(PneumaticsModuleType.CTREPCM, 4);
      public static Solenoid CLAW_OPEN = new Solenoid(PneumaticsModuleType.CTREPCM, 2);
      public static Solenoid ARM_OPEN = new Solenoid(PneumaticsModuleType.CTREPCM, 1);


      public record RPistonControl(Solenoid solenoid, DoubleSolenoid doubleSolenoid,
                                   double pulseDuration, long delay) {

      }


    }

    public static class ControlsConstants {

      public static class ArmConstants {

        public static final int kMotorPort = 4;

        public static final double kP = 1;

        public static final double ARM_S_VOLTS = 1;
        public static final double ARM_G_VOLTS = 1;
        public static final double ARM_VOLT_SEC_PER_RAD = 1;
        public static final double ARM_VOLT_SEC_SQUARED_PER_RAD = 0.1;
        public static final double MAX_VELOCITY_RAD_PER_SEC = 3;
        public static final double MAX_ACCELERATION_RAD_PER_SEC_SQUARED = 10;
        public static final double ARM_OFFSET_RADS = 0.5;
      }


      public static RobotAlignment ALIGNMENT = new RobotAlignment(new PIDController(0.0, 0.0, 0.0),
          RobotNav.getGyro().getFusedHeading());
    }

    public static double STAB_PID_P = .2;

    public static double STAB_PID_I = 0;
    public static double STAB_PID_D = 0.0;

    public static double BALANCE_PID_P = .02;
    public static double BALANCE_PID_I = 0.0;
    public static double BALANCE_PID_D = 0.0;

  }
}



