package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.RobotNav;
import java.util.List;

public class PathFollowCommand extends CommandBase {
private Drivetrain _drivetrain;
  public PathFollowCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _drivetrain = new Drivetrain();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }
  public Command getFollowCommand(){
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                RobotConstants.VOLTS_MAX,
                RobotConstants.VOLTS_SECONDS_PER_METER,
                RobotConstants.VOLTS_SECONDS_SQ_PER_METER),
            RobotConstants.DIFFERENTIAL_DRIVE_KINEMATICS,
            10);

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
            RobotConstants.AUTO_MAX_SPEED,
            RobotConstants.AUTO_MAX_ACCEL)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(RobotConstants.DIFFERENTIAL_DRIVE_KINEMATICS)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow.  All units in meters.
    Trajectory pathTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            // Pass config
            config);
//https://docs.wpilib.org/en/stable/docs/software/pathplanning/trajectory-tutorial/creating-drive-subsystem.html#voltage-based-drive-method
    //https://docs.wpilib.org/en/stable/docs/software/pathplanning/trajectory-tutorial/creating-following-trajectory.html
    RamseteCommand ramseteCommand =
        new RamseteCommand(
            pathTrajectory,
            RobotNav::get_robotPose2d,
            new RamseteController(RobotConstants.RAMSETE_B, RobotConstants.RAMSETE_ZETA),
            new SimpleMotorFeedforward(
                RobotConstants.VOLTS_MAX,
                RobotConstants.VOLTS_SECONDS_PER_METER,
                RobotConstants.VOLTS_SECONDS_SQ_PER_METER),
            RobotConstants.DIFFERENTIAL_DRIVE_KINEMATICS,
            Drivetrain::get_wheelSpeeds,
            new PIDController(DriveConstants.kPDriveVel, 0, 0),
            new PIDController(DriveConstants.kPDriveVel, 0, 0),
            // RamseteCommand passes volts to the callback
            _drivetrain::setDriveVolts,
            _drivetrain);

    // Reset odometry to the starting pose of the trajectory.
    _drivetrain.resetOdometry(pathTrajectory.getInitialPose());
    m_robotDrive.resetOdometry(pathTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> _drivetrain.setDriveVolts(0, 0));
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }


}
