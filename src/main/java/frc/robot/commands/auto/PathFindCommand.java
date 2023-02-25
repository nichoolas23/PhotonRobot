package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.Field.RoboField;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utilities.RobotNav;
import java.util.List;

public class PathFindCommand extends CommandBase {

private Drivetrain _drivetrain;
private Pose2d _target;


/**
* Command that finds a path from the robot's current position to the _target position and sends it to the drive train.
*/
  public PathFindCommand(Drivetrain drivetrain, Pose2d target) {
    _drivetrain = drivetrain;
    this._target = target;

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

   Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        RobotNav.get_diffDrivePose().getEstimatedPosition(), List.of(),
       _target,
        new TrajectoryConfig(2, 2)
    );
    RoboField.putTraj(trajectory);

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
