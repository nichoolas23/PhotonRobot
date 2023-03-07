package frc.robot.commands.arm;

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

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return  _armSubSystem.setArmGoalCommand(400);
  }

}
