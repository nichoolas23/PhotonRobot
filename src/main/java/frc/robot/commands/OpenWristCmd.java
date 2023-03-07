package frc.robot.commands;

import static frc.robot.Constants.RobotConstants.PneumaticsConstants.CLAW_OPEN;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pneumatics;

public class OpenWristCmd extends CommandBase {

  private final Pneumatics _pneumatics;

  public OpenWristCmd(Pneumatics pneumatics) {
    // Use addRequirements() here to declare subsystem dependencies.
    _pneumatics = pneumatics;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return _pneumatics.setPiston(true,CLAW_OPEN);
  }

}
