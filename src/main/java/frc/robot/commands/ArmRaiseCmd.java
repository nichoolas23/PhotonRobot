package frc.robot.commands;

import static frc.robot.PhysicalInputs.XBOX_CONTROLLER;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ArmRaiseCmd extends CommandBase {

  private final Arm _armSubSystem;

  /**
   * Creates a new ArmRaiseCmd.
   */

  public ArmRaiseCmd(Arm armSubSystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    _armSubSystem = armSubSystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
  _armSubSystem.setArmGoalCommand(400);
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _armSubSystem.armStop();
    DriverStation.reportError("Raised",false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return XBOX_CONTROLLER.getLeftY() > .2;
  }

}
