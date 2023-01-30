package frc.robot.commands;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.RobotConstants.PneumaticsConstants.RPiston;
import frc.robot.subsystems.Pneumatics;


/**
* Command used to control any piston on the robot dynamically.
 *  This will be used for any inputs outside standard open/close up down where finer control is needed.
*/
public class PistonExtendCmd extends CommandBase {
  private final RPiston[] _pistons;
  public PistonExtendCmd(RPiston... pistons) {

    _pistons = pistons;
    addRequirements();
  }
  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    for (RPiston piston : _pistons) {
      Pneumatics.setPiston(piston);

    }
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {

    return false;
  }
}

