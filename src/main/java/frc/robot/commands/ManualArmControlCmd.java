package frc.robot.commands;

import static frc.robot.PhysicalInputs.XBOX_CONTROLLER;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class ManualArmControlCmd extends CommandBase {
  private static WPI_TalonSRX _motorcontrollerLEFT = new WPI_TalonSRX(2);
  private static WPI_TalonSRX _motorcontrollerRIGHT = new WPI_TalonSRX(6);
  private static MotorControllerGroup _motorcontroller = new MotorControllerGroup(_motorcontrollerLEFT, _motorcontrollerRIGHT);

  public ManualArmControlCmd() {
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements();
  }

  @Override
  public void execute()
  {

  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DriverStation.reportError("Raised",false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return XBOX_CONTROLLER.getLeftY() > .2;
  }
}
