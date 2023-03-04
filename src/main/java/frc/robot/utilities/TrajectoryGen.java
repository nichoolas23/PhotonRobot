package frc.robot.utilities;

import static frc.robot.Constants.RobotConstants.AUTO_MAX_ACCEL;
import static frc.robot.Constants.RobotConstants.AUTO_MAX_SPEED;
import static frc.robot.Constants.RobotConstants.DRIVE_KINEMATICS;
import static frc.robot.Constants.RobotConstants.P_GAIN_DRIVE_VEL;
import static frc.robot.Constants.RobotConstants.RAMSETE_B;
import static frc.robot.Constants.RobotConstants.RAMSETE_ZETA;
import static frc.robot.Constants.RobotConstants.LEFT_VOLTS_MAX;
import static frc.robot.Constants.RobotConstants.LEFT_VOLTS_SECONDS_PER_METER;
import static frc.robot.Constants.RobotConstants.LEFT_VOLTS_SECONDS_SQ_PER_METER;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.Field.RoboField;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.Drivetrain;
import java.util.List;

public class TrajectoryGen {

  public static Command getTrajCmd(Drivetrain drivetrain, AprilTag tag,List<Translation2d> waypoints) {


    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                LEFT_VOLTS_MAX,
                LEFT_VOLTS_SECONDS_PER_METER,
                LEFT_VOLTS_SECONDS_SQ_PER_METER),
            DRIVE_KINEMATICS,
            10);

    // Create TRAJ_CONFIG for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
            AUTO_MAX_SPEED,
            AUTO_MAX_ACCEL)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DRIVE_KINEMATICS)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow.  All units in meters.
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        RobotNav.get_diffDrivePose().getEstimatedPosition(), List.of(),
        tag.pose.toPose2d(),
        config
    );
    RoboField.putTraj(trajectory);

    RamseteCommand ramseteCommand =
        new RamseteCommand(
            trajectory,
            RobotNav::getEstPose,
            new RamseteController(RAMSETE_B, RAMSETE_ZETA),
            new SimpleMotorFeedforward(
                LEFT_VOLTS_MAX,
                LEFT_VOLTS_SECONDS_PER_METER,
                LEFT_VOLTS_SECONDS_SQ_PER_METER),
            DRIVE_KINEMATICS,
            drivetrain::getWheelSpeeds,
            new PIDController(P_GAIN_DRIVE_VEL, 0, 0),
            new PIDController(P_GAIN_DRIVE_VEL, 0, 0),
            // RamseteCommand passes volts to the callback
            drivetrain::setVoltages,
            drivetrain);

    // Reset odometry to the starting pose of the trajectory.
    drivetrain.resetOdometry(trajectory.getInitialPose());
    RoboField.putTraj(trajectory);
    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> drivetrain.setVoltages(0, 0));
  }
  }



