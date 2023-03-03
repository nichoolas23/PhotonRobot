package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.RobotConstants.PneumaticsConstants.RPistonControl;
import frc.robot.subsystems.Pneumatics;


/**
* Command used to control any piston on the robot dynamically.
 *  This will be used for any inputs outside standard open/close up down where finer control is needed.
*/
public class PistonExtendCmd extends CommandBase {
  private final RPistonControl[] _pistons;
  public PistonExtendCmd(RPistonControl... pistons) {
    _pistons = pistons;

    addRequirements();
  }
  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    for (RPistonControl piston : _pistons) {
      //Pneumatics.setPiston(piston);

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

