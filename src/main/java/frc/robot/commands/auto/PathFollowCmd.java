package frc.robot.commands.auto;

import static frc.robot.Constants.RobotConstants.DRIVE_KINEMATICS;
import static frc.robot.Constants.RobotConstants.P_GAIN_DRIVE_VEL;
import static frc.robot.Constants.RobotConstants.RAMSETE_B;
import static frc.robot.Constants.RobotConstants.RAMSETE_ZETA;
import static frc.robot.Constants.RobotConstants.VOLTS_MAX;
import static frc.robot.Constants.RobotConstants.VOLTS_SECONDS_PER_METER;
import static frc.robot.Constants.RobotConstants.VOLTS_SECONDS_SQ_PER_METER;
import static frc.robot.Constants.RobotConstants.TRAJ_CONFIG;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.Field.RoboField;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utilities.RobotNav;
import frc.robot.utilities.TrajectoryGen;
import java.util.List;

public class PathFollowCmd extends CommandBase {

  private final Drivetrain _drivetrain;
  /**
   * Creates a new PathFollowCmd.
   */
  public PathFollowCmd(Drivetrain drivetrain) {
    _drivetrain = drivetrain;
    // Use addRequirements() here to declare subsystem dependencies.
addRequirements();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    TrajectoryGen.getTrajCmd(_drivetrain).schedule();

    // An example trajectory to follow.  All units in meters.
   /* Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        RobotNav.get_diffDrivePose().getEstimatedPosition(), List.of(),
        FieldConstants.FIRST_BLUE_GRID,
        TRAJ_CONFIG
    );
    RoboField.putTraj(trajectory);
    RamseteCommand ramseteCommand =
        new RamseteCommand(
            trajectory,
            RobotNav::getEstPose,
            new RamseteController(RAMSETE_B, RAMSETE_ZETA),
            new SimpleMotorFeedforward(
                VOLTS_MAX,
                VOLTS_SECONDS_PER_METER,
                VOLTS_SECONDS_SQ_PER_METER),
            DRIVE_KINEMATICS,
            _drivetrain::getWheelSpeeds,
            new PIDController(P_GAIN_DRIVE_VEL, 0, 0),
            new PIDController(P_GAIN_DRIVE_VEL, 0, 0),
            // RamseteCommand passes volts to the callback
            _drivetrain::setVoltages,
            _drivetrain);
    var toRun = ramseteCommand.andThen(() ->_drivetrain.setVoltages(0, 0));
    toRun.schedule();



    // Reset odometry to the starting pose of the trajectory.
    _drivetrain.resetOdometry(trajectory.getInitialPose());
    RoboField.putTraj(trajectory);
    // Run path following command, then stop at the end.
*/
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
