package frc.robot.subsystems;

import static frc.robot.Constants.RobotConstants.TRACK_WIDTH;
import static frc.robot.Constants.VisionConstants.POSE_ESTIMATOR;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.GlobalPoseCalc;
import frc.robot.utilities.RobotNav;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;

public class Drivetrain extends SubsystemBase {


// Start motor setup
private AHRS _gyro = new AHRS();
  private final WPI_TalonSRX[] wpi_talonSRXES = new WPI_TalonSRX[]{new WPI_TalonSRX(1),
      new WPI_TalonSRX(3), new WPI_TalonSRX(2), new WPI_TalonSRX(4)};
  private final MotorControllerGroup _leftDrive = new MotorControllerGroup(wpi_talonSRXES[0], wpi_talonSRXES[1]);
  private final MotorControllerGroup _rightDrive = new MotorControllerGroup(wpi_talonSRXES[2], wpi_talonSRXES[3]);
  private final DifferentialDriveKinematics _kinematics =
      new DifferentialDriveKinematics(TRACK_WIDTH);
  private final DifferentialDrivePoseEstimator _diffPoseEstimator =
      new DifferentialDrivePoseEstimator(
          _kinematics, _gyro.getRotation2d(), 0.0, 0.0, new Pose2d());

  private GlobalPoseCalc _globalPoseCalc = new GlobalPoseCalc();

  //private EncoderSim _leftEncoder = new EncoderSim(new Encoder(EncoderType.kQuadrature, 0, 1)));
  //private EncoderSim _rightEncoder = new EncoderSim(1);

  // End motor setup



  //  Start DifferentialDrive setup
  private final DifferentialDrive _differentialDrive = new DifferentialDrive(_leftDrive,
      _rightDrive);

  public Drivetrain() {
    //wpi_talonSRXES[1].getfee

    _rightDrive.setInverted(true);
    _differentialDrive.setSafetyEnabled(false);

    _diffPoseEstimator.update(
        _gyro.getRotation2d(), 0, 0);
    //_globalPoseCalc = new GlobalPoseCalc();
  }
  @Override
  public void periodic() {
      //_globalPoseCalc.getEstimatedGlobalPose(POSE_ESTIMATOR.getReferencePose().toPose2d());
  }
  public void updateOdometry() {
_diffPoseEstimator.update(
        _gyro.getRotation2d(), 0, 0);

    // Also apply vision measurements. We use 0.3 seconds in the past as an example
    // -- on
    // a real robot, this must be calculated based either on latency or timestamps.
    Optional<EstimatedRobotPose> result = _globalPoseCalc.getEstimatedGlobalPose(_diffPoseEstimator.getEstimatedPosition());

    if (result.isPresent()) {
      RobotNav.set_estimatedRobotPose(result.get());
      EstimatedRobotPose camPose = RobotNav.get_estimatedRobotPose();
      _diffPoseEstimator.addVisionMeasurement(camPose.estimatedPose.toPose2d(), camPose.timestampSeconds);

    } else {
      // move it way off the screen to make it disappear
          }



  }



/**
* Controls the driving mechanism of the robot.
 * @param forwardSpeed Between 0 and 1.0
 * @param reverseSpeed Between 0 and 1.0 inverted for driving backwards
 * @param rot Between -1.0 and 1.0 for turning
*/
  public void drive(double forwardSpeed, double reverseSpeed, double rot, boolean isAuto,boolean isTracking) {

    _differentialDrive.arcadeDrive(reverseSpeed > 0 ? reverseSpeed*-1 : forwardSpeed, rot*-1);
    RamseteController ramseteController = new RamseteController();
  }


}
