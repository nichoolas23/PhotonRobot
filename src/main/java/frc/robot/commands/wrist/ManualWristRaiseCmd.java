package frc.robot.commands.wrist;

import static frc.robot.PhysicalInputs.XBOX_CONTROLLER;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Wrist;


public class ManualWristRaiseCmd extends CommandBase {

  private final Wrist wrist;
  private static WPI_TalonSRX _motorcontrollerWRIST = new WPI_TalonSRX(3);
  public ManualWristRaiseCmd(Wrist wrist) {
    this.wrist = wrist;

    addRequirements(this.wrist);
  }

  @Override
  public void initialize() {

  }


  @Override
  public void execute() {
      wrist.controlWristVelocity(XBOX_CONTROLLER.getLeftY() * 500);
  }



  @Override
  public boolean isFinished() {
    // TODO: Make this return true when this Command no longer needs to run execute()
    return false;
  }


  @Override
  public void end(boolean interrupted) {

  }
}
