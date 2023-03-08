package frc.robot.subsystems;

import static frc.robot.Constants.RobotConstants.ControlsConstants.ArmConstants.ARM_G_VOLTS;
import static frc.robot.Constants.RobotConstants.ControlsConstants.ArmConstants.ARM_OFFSET_RADS;
import static frc.robot.Constants.RobotConstants.ControlsConstants.ArmConstants.ARM_S_VOLTS;
import static frc.robot.Constants.RobotConstants.ControlsConstants.ArmConstants.ARM_VOLT_SEC_PER_RAD;
import static frc.robot.Constants.RobotConstants.ControlsConstants.ArmConstants.ARM_VOLT_SEC_SQUARED_PER_RAD;
import static frc.robot.Constants.RobotConstants.ControlsConstants.ArmConstants.MAX_ACCELERATION_RAD_PER_SEC_SQUARED;
import static frc.robot.Constants.RobotConstants.ControlsConstants.ArmConstants.MAX_VELOCITY_RAD_PER_SEC;
import static frc.robot.Constants.RobotConstants.ControlsConstants.ArmConstants.kP;
import static frc.robot.Constants.RobotConstants.PneumaticsConstants.ARM_OPEN;
import static frc.robot.NickReplacementTroubleshooter.FEED_FORWARD_ARM;
import static frc.robot.NickReplacementTroubleshooter.FEED_FORWARD_ARM_EXTENDED;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.robot.utilities.ArmMotorController;

public class Arm extends SubsystemBase {
  /**
   * Creates a new Arm.
   */
  private static WPI_TalonSRX _motorcontrollerLEFT = new WPI_TalonSRX(2);
  private static WPI_TalonSRX _motorcontrollerRIGHT = new WPI_TalonSRX(6);
  private static MotorControllerGroup _motorcontroller = new MotorControllerGroup(_motorcontrollerLEFT, _motorcontrollerRIGHT);
  /** Create a new ArmSubsystem. */
  public Arm() {

  }
  public boolean setArmPosition(double targetPos) {

    int kMeasuredPosHorizontal = 926; //Position measured when arm is horizontal
    double kTicksPerDegree = 4096.0 / 360; //Sensor is 1:1 with arm rotation
    double currentPos = _motorcontrollerRIGHT.getSensorCollection().getQuadraturePosition();

    double degrees = (currentPos - kMeasuredPosHorizontal) / kTicksPerDegree;
    double radians = java.lang.Math.toRadians(degrees);
    double cosineScalar = java.lang.Math.cos(radians);
    _motorcontrollerLEFT.setInverted(false);
    _motorcontrollerRIGHT.setInverted(true);
    double maxGravityFF = 0;
    if(ARM_OPEN.get()){

      maxGravityFF = FEED_FORWARD_ARM_EXTENDED;
    }
    else{
      maxGravityFF = FEED_FORWARD_ARM;
    }
    SmartDashboard.putNumber("Arm Gravity",(maxGravityFF * cosineScalar));
    _motorcontrollerRIGHT.set(
        ControlMode.Velocity, targetPos, DemandType.ArbitraryFeedForward, (maxGravityFF * cosineScalar));
    _motorcontrollerLEFT.set(ControlMode.Follower,6);

  return true;
  }


  public boolean setArmGoalCommand(double targetPos) {
    _motorcontrollerRIGHT.setSensorPhase(true);


    int kMeasuredPosHorizontal = 926; //Position measured when arm is horizontal
    double kTicksPerDegree = 4096.0 / 360; //Sensor is 1:1 with arm rotation
    double currentPos = _motorcontrollerRIGHT.getSensorCollection().getQuadraturePosition();

    double degrees = (currentPos - kMeasuredPosHorizontal) / kTicksPerDegree;
    double radians = java.lang.Math.toRadians(degrees);
    double cosineScalar = java.lang.Math.cos(radians);

    _motorcontrollerLEFT.setInverted(false);
    _motorcontrollerRIGHT.setInverted(true);
    double maxGravityFF = 0.16;
    DriverStation.reportError("yes",false);

    //_motorcontrollerLEFT.getActiveTrajectoryArbFeedFwd();
    //, DemandType.ArbitraryFeedForward, maxGravityFF * cosineScalar
    _motorcontrollerRIGHT.set(
        ControlMode.MotionMagic, targetPos, DemandType.ArbitraryFeedForward, (maxGravityFF * cosineScalar));
    _motorcontrollerLEFT.set(ControlMode.Follower,6);
    SmartDashboard.putNumber("left arm",_motorcontrollerLEFT.getActiveTrajectoryPosition());
    SmartDashboard.putNumber("right arm",-(maxGravityFF * cosineScalar));
    SmartDashboard.putNumber("left arm ff",_motorcontrollerLEFT.getActiveTrajectoryArbFeedFwd());
    SmartDashboard.putNumber("right armff",_motorcontrollerRIGHT.getActiveTrajectoryArbFeedFwd());
    return true;
  }
  public void armStop(){
    _motorcontrollerLEFT.stopMotor();
    _motorcontrollerRIGHT.stopMotor();
  }
}
