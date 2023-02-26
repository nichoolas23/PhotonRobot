package frc.robot.commands.auto;

import static frc.robot.Constants.RobotConstants.LEFT_ENCODER;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;

public class InitialDriveOutAutoCmd extends CommandBase {
  private PIDController _controller = new PIDController(0.1, 0, 0);

  /**
   * Creates a new InitialDriveOutAutoCmd.
   */
  public InitialDriveOutAutoCmd()
  {

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {


  //Get encoder data

    //LEFT_ENCODER.getDistance();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false ;
  }

}
