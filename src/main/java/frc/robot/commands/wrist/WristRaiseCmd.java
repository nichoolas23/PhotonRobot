package frc.robot.commands.wrist;

import static frc.robot.PhysicalInputs.XBOX_CONTROLLER;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Wrist;

public class WristRaiseCmd extends CommandBase {


  private Wrist _wristSubSystem;


  /**
   * Creates a new WristRaiseCmd.
   */
  public WristRaiseCmd(Wrist wristSubSystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    _wristSubSystem = wristSubSystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }


  @Override
  public void execute() {

  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //DriverStation.reportError("Raised", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return _wristSubSystem.controlWrist(200);
  }
}
