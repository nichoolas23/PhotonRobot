package frc.robot.commands;

import static frc.robot.Constants.RobotConstants.PneumaticsConstants.LEFT_HILO_GEARSHIFT;
import static frc.robot.Constants.RobotConstants.PneumaticsConstants.RIGHT_HILO_GEARSHIFT;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pneumatics;

public class ChangeGearCmd extends CommandBase {

  private Pneumatics _pneumatics;

  /**
   * Creates a new ChangeGearCmd.
   */
  public ChangeGearCmd(Pneumatics pneumatics) {
    _pneumatics = pneumatics;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(pneumatics);
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
    return _pneumatics.setPiston( LEFT_HILO_GEARSHIFT,RIGHT_HILO_GEARSHIFT);
  }

}
