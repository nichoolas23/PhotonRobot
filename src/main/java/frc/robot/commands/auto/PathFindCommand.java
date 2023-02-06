package frc.robot.commands.auto;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.Field.RoboField;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utilities.RobotNav;
import java.util.List;

public class PathFindCommand extends CommandBase {

Drivetrain _drivetrain = new Drivetrain();


/**
* Command that finds a path from the robot's current position to the target position and sends it to the drive train.
*/
  public PathFindCommand() {

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
        RobotNav.getEstimatedRobotPose().estimatedPose.toPose2d(), List.of(),
        FieldConstants.FIFTH_BLUE_GRID,
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
