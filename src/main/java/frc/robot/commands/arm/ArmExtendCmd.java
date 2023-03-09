package frc.robot.commands.arm;

import static frc.robot.Constants.RobotConstants.PneumaticsConstants.ARM_OPEN;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pneumatics;


public class ArmExtendCmd extends CommandBase {

  private final Pneumatics _pneumatics;

  public ArmExtendCmd(Pneumatics _pneumatics) {
    this._pneumatics = _pneumatics;
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements();
  }

  /**
   * <p>
   * Returns whether this command has finished. Once a command finishes -- indicated by this method
   * returning true -- the scheduler will call its {@link #end(boolean)} method.
   * </p><p>
   * Returning false will result in the command never ending automatically. It may still be
   * cancelled manually or interrupted by another command. Hard coding this command to always return
   * true will result in the command executing once and finishing immediately. It is recommended to
   * use * {@link edu.wpi.first.wpilibj2.command.InstantCommand InstantCommand} for such an
   * operation.
   * </p>
   *
   * @return whether this command has finished.
   */
  @Override
  public boolean isFinished() {
    // TODO: Make this return true when this Command no longer needs to run execute()
    return _pneumatics.setPiston(ARM_OPEN);
  }


}
