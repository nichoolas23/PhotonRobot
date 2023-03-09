package frc.robot.commands.arm;

import static frc.robot.PhysicalInputs.XBOX_CONTROLLER;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;


public class ManualArmControlCmd extends CommandBase {

  private static WPI_TalonSRX _motorcontrollerLEFT = new WPI_TalonSRX(2);
  private static WPI_TalonSRX _motorcontrollerRIGHT = new WPI_TalonSRX(6);
  private static MotorControllerGroup _motorcontroller = new MotorControllerGroup(
      _motorcontrollerLEFT, _motorcontrollerRIGHT);
  private Arm _armSubSystem;

  public ManualArmControlCmd(Arm armSubSystem) {
    _armSubSystem = armSubSystem;
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements();
  }

  @Override
  public void execute() {

    _armSubSystem.setArmPosition(
       XBOX_CONTROLLER.getRightY()*-900);
  }


  /**
   * @param interrupted whether the command was interrupted/canceled
   *
   */
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
