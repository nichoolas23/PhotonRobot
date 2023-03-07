package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase {


  private static WPI_TalonSRX _motorcontrollerWRIST = new WPI_TalonSRX(3);

  public Wrist() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public boolean controlWristVelocity(double targetPos){

    int kMeasuredPosHorizontal = 591; //Position measured when arm is horizontal
    double kTicksPerDegree = 4096.0 / 360; //Sensor is 1:1 with arm rotation
    double currentPos = _motorcontrollerWRIST.getSensorCollection().getQuadraturePosition();

    double degrees = (currentPos - kMeasuredPosHorizontal) / kTicksPerDegree;
    double radians = java.lang.Math.toRadians(degrees);
    double cosineScalar = java.lang.Math.cos(radians);
    _motorcontrollerWRIST.setInverted(false);
    double maxGravityFF = 0.12;
    SmartDashboard.putNumber("WRIST GRAV",(maxGravityFF * cosineScalar));
    _motorcontrollerWRIST.set(
        ControlMode.Velocity, targetPos, DemandType.ArbitraryFeedForward, -(maxGravityFF * cosineScalar));

    return true;
  }

  public boolean controlWrist(double targetPos) {

      //_motorcontrollerWRIST.setSensorPhase(true);


      int kMeasuredPosHorizontal = 250; //Position measured when arm is horizontal
      double kTicksPerDegree = 4096.0 / 360; //Sensor is 1:1 with arm rotation
      double currentPos = _motorcontrollerWRIST.getSensorCollection().getQuadraturePosition();
      _motorcontrollerWRIST.setInverted(false);
      double degrees = (currentPos - kMeasuredPosHorizontal) / kTicksPerDegree;
      double radians = java.lang.Math.toRadians(degrees);
      double cosineScalar = java.lang.Math.cos(radians);

      double maxGravityFF = 0.07;


      //_motorcontrollerLEFT.getActiveTrajectoryArbFeedFwd();
      //, DemandType.ArbitraryFeedForward, maxGravityFF * cosineScalar
      _motorcontrollerWRIST.set(
          ControlMode.MotionMagic, targetPos, DemandType.ArbitraryFeedForward,
          (maxGravityFF * cosineScalar));

      SmartDashboard.putNumber("Encoder Wrist", _motorcontrollerWRIST.getSelectedSensorPosition());
      return true;
    }
  }


