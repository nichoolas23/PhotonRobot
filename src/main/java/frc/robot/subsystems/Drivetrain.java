package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {


// Start motor setup
  private final WPI_TalonSRX[] wpi_talonSRXES = new WPI_TalonSRX[]{new WPI_TalonSRX(1),
      new WPI_TalonSRX(3), new WPI_TalonSRX(2), new WPI_TalonSRX(4)};
  private final MotorControllerGroup leftDrive = new MotorControllerGroup(wpi_talonSRXES[0], wpi_talonSRXES[1]);
  private final MotorControllerGroup rightDrive = new MotorControllerGroup(wpi_talonSRXES[2], wpi_talonSRXES[3]);

  public MotorControllerGroup getLeftDrive() {return leftDrive;}
  public MotorControllerGroup getRightDrive() {return rightDrive;}

  // End motor setup

  private Encoder _leftEncoder = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
  private Encoder _rightEncoder = new Encoder(2, 3, true, Encoder.EncodingType.k4X);

//  Start DifferentialDrive setup
  private final DifferentialDrive _differentialDrive = new DifferentialDrive(leftDrive, rightDrive);
  private final DifferentialDriveKinematics _kinematics = new DifferentialDriveKinematics(.5);
  private DifferentialDriveOdometry _odometry = new DifferentialDriveOdometry(RobotNav.get_gyro().getRotation2d(),_leftEncoder.getDistance(),_rightEncoder.getDistance());

  public Drivetrain() {
    rightDrive.setInverted(true);
  }

  @Override
  public void periodic() {
    updateOdometry();
  }

  public void updateOdometry() {
    var gyroAngle = RobotNav.get_gyro().getRotation2d();

    // Update the pose
    RobotNav.set_robotPose2d(_odometry.update(gyroAngle,
        _leftEncoder.getDistance(),
        _rightEncoder.getDistance()));
  }

  public void drive(double forwardSpeed, double reverseSpeed, double rot) {

    _differentialDrive.arcadeDrive(reverseSpeed > 0 ? reverseSpeed*-1 : forwardSpeed, rot);

  }




}
