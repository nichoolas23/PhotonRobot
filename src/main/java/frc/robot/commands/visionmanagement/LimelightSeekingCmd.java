package frc.robot.commands.visionmanagement;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utilities.LimelightHelpers;

public class LimelightSeekingCmd extends CommandBase {
/**
   * Creates a new LimelightSeekingCmd.
   */
  public LimelightSeekingCmd() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    LimelightHelpers.setPipelineIndex("seeking",0);

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
