package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.RobotConstants;

public class Drivetrain extends SubsystemBase {


// Start motor setup
  private final WPI_TalonSRX[] wpi_talonSRXES = new WPI_TalonSRX[]{new WPI_TalonSRX(1),
      new WPI_TalonSRX(3), new WPI_TalonSRX(2), new WPI_TalonSRX(4)};
  private final MotorControllerGroup leftDrive = new MotorControllerGroup(wpi_talonSRXES[0], wpi_talonSRXES[1]);
  private final MotorControllerGroup rightDrive = new MotorControllerGroup(wpi_talonSRXES[2], wpi_talonSRXES[3]);

  // End motor setup

  private Encoder _leftEncoder = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
   private Encoder _rightEncoder = new Encoder(2, 3, true, EncodingType.k1X);

  //  Start DifferentialDrive setup
  private final DifferentialDrive _differentialDrive = new DifferentialDrive(leftDrive, rightDrive);
  private DifferentialDriveWheelSpeeds _wheelSpeeds = new DifferentialDriveWheelSpeeds(0, 0);

  //  private DifferentialDriveOdometry _odometry = new DifferentialDriveOdometry(RobotNav.get_gyro().getRotation2d(),_leftEncoder.getDistance(),_rightEncoder.getDistance());

  public Drivetrain() {
    _leftEncoder.setDistancePerPulse(RobotConstants.DISTANCE_PER_PULSE);
    rightDrive.setInverted(true);
    _differentialDrive.setSafetyEnabled(false);
  }

  @Override
  public void periodic() {

  updateOdometry();
  }

  public void updateOdometry() {
    var gyroAngle = RobotNav.get_gyro().getRotation2d();
    _wheelSpeeds = new DifferentialDriveWheelSpeeds(_leftEncoder.getRate(), _rightEncoder.getRate());
  }

/**
* Controls the driving mechanism of the robot.
 * @param forwardSpeed Between 0 and 1.0
 * @param reverseSpeed Between 0 and 1.0 inverted for driving backwards
 * @param rot Between -1.0 and 1.0 for turning
*/
  public void drive(double forwardSpeed, double reverseSpeed, double rot) {

    _differentialDrive.arcadeDrive(reverseSpeed > 0 ? reverseSpeed*-1 : forwardSpeed, rot);


  }

  public DifferentialDriveWheelSpeeds get_wheelSpeeds() {
    return _wheelSpeeds;
  }


 /* public static DifferentialDriveWheelSpeeds get_WheelSpeed() {
    return new DifferentialDriveWheelSpeeds(Constants.RobotConstants.kMaxSpeedMetersPerSecond,
        Constants.RobotConstants.kMaxSpeedMetersPerSecond);
  }*/
}
