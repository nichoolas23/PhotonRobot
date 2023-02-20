package frc.robot.subsystems;

import static frc.robot.Constants.RobotConstants.DRIVE_KINEMATICS;
import static frc.robot.Constants.RobotConstants.LEFT_ENCODER;
import static frc.robot.Constants.RobotConstants.PhysicalConstants.WHEEL_CIRCUM;
import static frc.robot.Constants.RobotConstants.RIGHT_ENCODER;
import static frc.robot.Constants.VisionConstants.VISION_STD_DEV;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.DifferentialDriveWheelVoltages;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Field.RoboField;
import frc.robot.utilities.LimelightHelpers;
import frc.robot.utilities.RobotNav;
import org.opencv.core.Mat.Tuple2;
import org.opencv.core.Point;

public class Drivetrain extends SubsystemBase {
  private static boolean isCalibrated = false;
  private static boolean isAuto = false;


private static final WPI_TalonSRX[] wpi_talonSRXES = new WPI_TalonSRX[]{new WPI_TalonSRX(1),
      new WPI_TalonSRX(3), new WPI_TalonSRX(2), new WPI_TalonSRX(4)};
  private static final MotorControllerGroup _leftDrive = new MotorControllerGroup(wpi_talonSRXES[0],
      wpi_talonSRXES[1]);
  private static final MotorControllerGroup _rightDrive = new MotorControllerGroup(wpi_talonSRXES[2],
      wpi_talonSRXES[3]);


  /*private final WPI_TalonSRX[] wpi_talonSRXES = new WPI_TalonSRX[]{new WPI_TalonSRX(1),
      new WPI_TalonSRX(3), new WPI_TalonSRX(2), new WPI_TalonSRX(4)};
  private final MotorControllerGroup _leftDrive = new MotorControllerGroup(wpi_talonSRXES[1],
      wpi_talonSRXES[3]);
  private final MotorControllerGroup _rightDrive = new MotorControllerGroup(wpi_talonSRXES[0],
      wpi_talonSRXES[2]);*/
  private static final DifferentialDrive _differentialDrive = new DifferentialDrive(_leftDrive,
      _rightDrive);
  private static final AHRS _gyro = new AHRS();

  // Start motor setup
  private static DifferentialDrivePoseEstimator _diffPoseEstimator =
      new DifferentialDrivePoseEstimator(
          DRIVE_KINEMATICS, _gyro.getRotation2d(), 0.0, 0.0, new Pose2d());
  private static final DifferentialDriveWheelSpeeds _diffDriveWheelSpeeds = new DifferentialDriveWheelSpeeds(0,0);
  private static final DifferentialDriveWheelVoltages _diffDriveWheelVoltages = new DifferentialDriveWheelVoltages();
  private static final DifferentialDriveOdometry _diffDriveOdometry = new DifferentialDriveOdometry(_gyro.getRotation2d(),0,0);




  public Drivetrain() {

    LEFT_ENCODER.setDistancePerPulse(WHEEL_CIRCUM/ 357.75);
    LEFT_ENCODER.setReverseDirection(true);
    RIGHT_ENCODER.setDistancePerPulse(WHEEL_CIRCUM / 357.75);


    _rightDrive.setInverted(true);
    _differentialDrive.setSafetyEnabled(false);





    _diffPoseEstimator.setVisionMeasurementStdDevs(VISION_STD_DEV);

  }

  public static boolean isIsAuto() {
    return isAuto;
  }

  public static void setIsAuto(boolean isAuto) {
    Drivetrain.isAuto = isAuto;
  }


  @Override
  public void periodic() {




  }

  public void updateOdometry() {

    _diffDriveOdometry.update(_gyro.getRotation2d(), LEFT_ENCODER.getDistance(), RIGHT_ENCODER.getDistance());

    _diffDriveWheelSpeeds.leftMetersPerSecond = LEFT_ENCODER.getRate();
    _diffDriveWheelSpeeds.rightMetersPerSecond = RIGHT_ENCODER.getRate();

    _diffDriveWheelVoltages.left = wpi_talonSRXES[0].getMotorOutputVoltage() + wpi_talonSRXES[1].getMotorOutputVoltage();
    _diffDriveWheelVoltages.right = wpi_talonSRXES[3].getMotorOutputVoltage() + wpi_talonSRXES[2].getMotorOutputVoltage();

    _diffPoseEstimator.update(_gyro.getRotation2d(), LEFT_ENCODER.getDistance(), RIGHT_ENCODER.getDistance());
    SmartDashboard.putNumber("Left Encoder", LEFT_ENCODER.getDistance());
    SmartDashboard.putNumber("Right Encoder", RIGHT_ENCODER.getDistance());
    SmartDashboard.putNumber("Left Encoder Rate", LEFT_ENCODER.getRate());
    SmartDashboard.putNumber("Right Encoder Rate", RIGHT_ENCODER.getRate());

    // Also apply vision measurements. We use 0.3 seconds in the past as an example
    // -- on
    // a real robot, this must be calculated based either on latency or timestamps.
    if(LimelightHelpers.getTV("")){
      if(isCalibrated){
        if(RobotNav.getFieldAdjPose(LimelightHelpers.getBotPose2d("")).getTranslation().getDistance(_diffPoseEstimator.getEstimatedPosition().getTranslation()) < 1){

          _diffPoseEstimator.addVisionMeasurement(RobotNav.getFieldAdjPose(LimelightHelpers.getBotPose2d("")), Timer.getFPGATimestamp(),VISION_STD_DEV);
        }
      }else{
        _diffPoseEstimator.addVisionMeasurement(RobotNav.getFieldAdjPose(LimelightHelpers.getBotPose2d("")), Timer.getFPGATimestamp(),VISION_STD_DEV);
      }

    }

    RoboField.fieldUpdate(_diffPoseEstimator.getEstimatedPosition());
    RobotNav.set_diffDrivePose(_diffPoseEstimator);
  }
  public void setBrakeMode(){
    for(var motor : wpi_talonSRXES){
      motor.setNeutralMode(NeutralMode.Coast);
    }
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {

    return _diffDriveWheelSpeeds;
  }
  public void setVoltages(double leftVolts, double rightVolts) {
    _leftDrive.setVoltage(leftVolts*.25);
    _rightDrive.setVoltage(rightVolts*.25);
  }
  public void resetOdometry(Pose2d initialPose) {
    resetEncoders();
    _diffDriveOdometry.resetPosition(
        _gyro.getRotation2d(),
        LEFT_ENCODER.getDistance(),
        RIGHT_ENCODER.getDistance(),
        initialPose);
  }

  public void resetEncoders() {
    LEFT_ENCODER.reset();
    RIGHT_ENCODER.reset();
  }


  /**
   * Controls the driving mechanism of the robot.
   *
   * @param forwardSpeed Between 0 and 1.0
   * @param reverseSpeed Between 0 and 1.0 inverted for driving backwards
   * @param rot          Between -1.0 and 1.0 for turning
   */
  public void drive(double forwardSpeed, double reverseSpeed, double rot, boolean isAuto,
      boolean isTracking) {
if(!this.isAuto){
  _differentialDrive.arcadeDrive(reverseSpeed > 0 ? reverseSpeed * -1 : forwardSpeed, rot * -1);
}



  }



}
