package frc.robot.commands;

import static frc.robot.Constants.RobotConstants.PnuematicsConstants.PistonSelect.WRIST_PISTON;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.RobotConstants.PnuematicsConstants.PistonSelect;

public class PistonExtendCmd extends CommandBase {

  private record Piston(int channel, double pulseDuration, boolean isExtend) {}
  private Piston _
  public PistonExtendCmd(PistonSelect config) {
    switch(config){
      case WRIST_PISTON:
        new Piston(0,2.0,true);
        break;
      case ARM_PISTON:
        break;
      default:
        throw new IllegalStateException("Unexpected value: " + config);
    }
    // Use addRequirements() here to declare subsystem dependencies.
  }
  @Override
  public void initialize() {
  }

  @Override
  public void execute() {

  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {

    return false;
  }
}

