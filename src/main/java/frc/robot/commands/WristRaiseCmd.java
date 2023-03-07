package frc.robot.commands;

import static frc.robot.PhysicalInputs.XBOX_CONTROLLER;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class WristRaiseCmd extends CommandBase {

  private static WPI_TalonSRX _motorcontrollerWRIST = new WPI_TalonSRX(3);


  /**
   * Creates a new WristRaiseCmd.
   */
  public WristRaiseCmd() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }


  @Override
  public void execute() {
    //_motorcontrollerWRIST.setSensorPhase(true);
    double targetPos = 725;

    int kMeasuredPosHorizontal = 725; //Position measured when arm is horizontal
    double kTicksPerDegree = 4096.0 / 360; //Sensor is 1:1 with arm rotation
    double currentPos = _motorcontrollerWRIST.getSensorCollection().getQuadraturePosition();
    _motorcontrollerWRIST.setInverted(false);
    double degrees = (currentPos - kMeasuredPosHorizontal) / kTicksPerDegree;
    double radians = java.lang.Math.toRadians(degrees);
    double cosineScalar = java.lang.Math.cos(radians);

    double maxGravityFF = 0.07;
    DriverStation.reportError("yes", false);

    //_motorcontrollerLEFT.getActiveTrajectoryArbFeedFwd();
    //, DemandType.ArbitraryFeedForward, maxGravityFF * cosineScalar
    _motorcontrollerWRIST.set(
        ControlMode.MotionMagic, targetPos, DemandType.ArbitraryFeedForward,
        (maxGravityFF * cosineScalar));

SmartDashboard.putNumber("Encoder Wrist",_motorcontrollerWRIST.getSelectedSensorPosition());
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //DriverStation.reportError("Raised", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
