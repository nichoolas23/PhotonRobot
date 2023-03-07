package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase {

  private DigitalInput _wristLimitSwitch = new DigitalInput(0);
  private WPI_TalonSRX _wristMotor = new WPI_TalonSRX(3);

  public Wrist() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void controlWrist(){
   /* double targetPos = 0;
    int kMeasuredPosHorizontal = 840; //Position measured when arm is horizontal
    double kTicksPerDegree = 4096.0 / 360; //Sensor is 1:1 with arm rotation
    double currentPos = _motorcontrollerRIGHT.getSelectedSensorPosition();
    double degrees = (currentPos - kMeasuredPosHorizontal) / kTicksPerDegree;
    double radians = java.lang.Math.toRadians(degrees);
    double cosineScalar = java.lang.Math.cos(radians);
    _motorcontrollerRIGHT.setInverted(true);
    double maxGravityFF = 0.07;
    _motorcontrollerLEFT.set(ControlMode.MotionMagic, targetPos, DemandType.ArbitraryFeedForward, maxGravityFF * cosineScalar);
    _motorcontrollerRIGHT.set(ControlMode.MotionMagic, targetPos, DemandType.ArbitraryFeedForward, maxGravityFF * cosineScalar);*/
  }

}
