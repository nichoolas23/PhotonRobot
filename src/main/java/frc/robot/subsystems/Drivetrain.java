package frc.robot.subsystems;

import static frc.robot.Constants.RobotConstants.DRIVE_KINEMATICS;
import static frc.robot.Constants.VisionConstants.VISION_STD_DEV;
import static frc.robot.utilities.RobotNav.getFieldAdjPose;
import static frc.robot.utilities.RobotNav.getLeftEncoderPosition;
import static frc.robot.utilities.RobotNav.getLeftEncoderVelocity;
import static frc.robot.utilities.RobotNav.getRightEncoderPosition;
import static frc.robot.utilities.RobotNav.getRightEncoderVelocity;
import static frc.robot.utilities.RobotNav.get_leftEncoder;
import static frc.robot.utilities.RobotNav.get_rightEncoder;
import static frc.robot.utilities.RobotNav.set_diffDrivePose;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.controller.DifferentialDriveWheelVoltages;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Field.RoboField;
import frc.robot.utilities.LimelightHelpers;
import frc.robot.utilities.RobotNav;

public class Drivetrain extends SubsystemBase {

  private static final boolean isCalibrated = false;


  /*  private static final WPI_TalonSRX[] wpi_talonSRXES = new WPI_TalonSRX[]{new WPI_TalonSRX(1),
        new WPI_TalonSRX(3), new WPI_TalonSRX(2), new WPI_TalonSRX(4)};
    private static final MotorControllerGroup _leftDrive = new MotorControllerGroup(wpi_talonSRXES[0],
        wpi_talonSRXES[1]);
    private static final MotorControllerGroup _rightDrive = new MotorControllerGroup(
        wpi_talonSRXES[2],
        wpi_talonSRXES[3]);*/

  public static final WPI_TalonSRX[] wpi_talonSRXES = {new WPI_TalonSRX(0),
      new WPI_TalonSRX(1), new WPI_TalonSRX(2), new WPI_TalonSRX(3), new WPI_TalonSRX(4),
      new WPI_TalonSRX(5), new WPI_TalonSRX(6), new WPI_TalonSRX(7), new WPI_TalonSRX(8),
      new WPI_TalonSRX(9), new WPI_TalonSRX(10), new WPI_TalonSRX(11), new WPI_TalonSRX(12),
      new WPI_TalonSRX(13), new WPI_TalonSRX(14), new WPI_TalonSRX(15)};
  private static final MotorControllerGroup _leftDrive = new MotorControllerGroup(wpi_talonSRXES[0],
      wpi_talonSRXES[1]);
  private static final MotorControllerGroup _rightDrive = new MotorControllerGroup(
      wpi_talonSRXES[4],
      wpi_talonSRXES[5]);


  /*  private static final WPI_TalonSRX[] wpi_talonSRXES = new WPI_TalonSRX[]{new WPI_TalonSRX(1),
         new WPI_TalonSRX(3), new WPI_TalonSRX(2), new WPI_TalonSRX(4)};
     private static final MotorControllerGroup _leftDrive = new MotorControllerGroup(wpi_talonSRXES[1],
         wpi_talonSRXES[3]);
     private static final MotorControllerGroup _rightDrive = new MotorControllerGroup(wpi_talonSRXES[0],
         wpi_talonSRXES[2]);*/
  private static final DifferentialDrive _differentialDrive = new DifferentialDrive(_leftDrive,
      _rightDrive);
  private static final AHRS _gyro = new AHRS();

  // Start motor setup
  private static final DifferentialDrivePoseEstimator _diffPoseEstimator =
      new DifferentialDrivePoseEstimator(
          DRIVE_KINEMATICS, _gyro.getRotation2d(), 0.0, 0.0, new Pose2d());
  private static final DifferentialDriveWheelSpeeds _diffDriveWheelSpeeds = new DifferentialDriveWheelSpeeds(
      0, 0);
  private static final DifferentialDriveWheelVoltages _diffDriveWheelVoltages = new DifferentialDriveWheelVoltages();
  private static final DifferentialDriveOdometry _diffDriveOdometry = new DifferentialDriveOdometry(
      _gyro.getRotation2d(), 0, 0);


  public Drivetrain() {

   /* LEFT_ENCODER.setQuadraturePosition(WHEEL_CIRCUM / 357.75);
    LEFT_ENCODER.(true);
    RIGHT_ENCODER.setDistancePerPulse(WHEEL_CIRCUM / 357.75);*/

    _rightDrive.setInverted(true);
    _differentialDrive.setSafetyEnabled(false);
    _diffPoseEstimator.setVisionMeasurementStdDevs(VISION_STD_DEV);
//this.setDefaultCommand(new ControllerDriveCmd(this,new XboxController(0)));
  }


  @Override
  public void periodic() {

  }


  public void updateOdometry() {

    _diffDriveOdometry.update(_gyro.getRotation2d(), getLeftEncoderPosition(),
        getRightEncoderPosition());

    _diffDriveWheelSpeeds.leftMetersPerSecond = getLeftEncoderVelocity();
    _diffDriveWheelSpeeds.rightMetersPerSecond = getRightEncoderVelocity();

    _diffDriveWheelVoltages.left =
        wpi_talonSRXES[0].getMotorOutputVoltage() + wpi_talonSRXES[1].getMotorOutputVoltage();
    _diffDriveWheelVoltages.right =
        wpi_talonSRXES[3].getMotorOutputVoltage() + wpi_talonSRXES[2].getMotorOutputVoltage();

    _diffPoseEstimator.update(_gyro.getRotation2d(), getLeftEncoderPosition(),
        getRightEncoderPosition());
    SmartDashboard.putNumber("Left Encoder", getLeftEncoderPosition());
    SmartDashboard.putNumber("Right Encoder", getRightEncoderPosition());
    SmartDashboard.putNumber("Left Encoder Rate", getLeftEncoderVelocity());
    SmartDashboard.putNumber("Right Encoder Rate", getRightEncoderVelocity());

    if (LimelightHelpers.getTV("")) {
      if (isCalibrated) {
        if (getFieldAdjPose(LimelightHelpers.getBotPose2d("")).getTranslation()
            .getDistance(_diffPoseEstimator.getEstimatedPosition().getTranslation()) < .4) {

          _diffPoseEstimator.addVisionMeasurement(
              getFieldAdjPose(LimelightHelpers.getBotPose2d("")), Timer.getFPGATimestamp(),
              VISION_STD_DEV);
        }
      } else {
        _diffPoseEstimator.addVisionMeasurement(
            getFieldAdjPose(LimelightHelpers.getBotPose2d("")), Timer.getFPGATimestamp(),
            VISION_STD_DEV);
      }

    }

    RoboField.fieldUpdate(_diffPoseEstimator.getEstimatedPosition());
    set_diffDrivePose(_diffPoseEstimator);
  }

  public void setBrakeMode() {
    for (var motor : wpi_talonSRXES) {
      motor.setNeutralMode(NeutralMode.Brake);
    }
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {

    return _diffDriveWheelSpeeds;
  }
  public void emergencyStop() {
    for (var motor : wpi_talonSRXES) {
      motor.set(0);
    }
    _leftDrive.stopMotor();
    _rightDrive.stopMotor();
    CommandScheduler.getInstance().cancelAll();
    throw new RuntimeException("EMERGENCY STOP");
  }

  public void setVoltages(double leftVolts, double rightVolts) {
    _leftDrive.setVoltage(leftVolts * .25);
    _rightDrive.setVoltage(rightVolts * .25);
  }

  public void resetOdometry(Pose2d initialPose) {
    resetEncoders();
    _diffDriveOdometry.resetPosition(
        _gyro.getRotation2d(),
        RobotNav.getLeftEncoderPosition(),
        RobotNav.getRightEncoderPosition(),
        initialPose);
  }

  public void resetEncoders() {

    get_leftEncoder().setQuadraturePosition(0, 0);
    get_rightEncoder().setQuadraturePosition(0, 0);
  }


  /**
   * Controls the driving mechanism of the robot.
   *
   * @param forwardSpeed Between 0 and 1.0
   * @param reverseSpeed Between 0 and 1.0 inverted for driving backwards
   * @param rot          Between -1.0 and 1.0 for turning
   */
  public void drive(double forwardSpeed, double reverseSpeed, double rot) {

    _differentialDrive.arcadeDrive(reverseSpeed > 0 ? reverseSpeed * -1 : forwardSpeed, rot * -1);


  }


  public void drive(double speed, double rot) {

    _differentialDrive.arcadeDrive(speed, rot);
  }
}
