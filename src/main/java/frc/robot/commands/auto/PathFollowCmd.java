package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utilities.TrajectoryGen;

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
