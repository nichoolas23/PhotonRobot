package frc.robot.commands.auto;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utilities.LimelightHelpers;
import frc.robot.utilities.RobotNav;


public class AlignWithBlockGridCmd extends CommandBase {

  final double ANGULAR_P = 0.2;
  final double ANGULAR_D = 0;

  private final XboxController _controller;
  private PIDController _turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);
  private Drivetrain _drivetrain;
  private Pose3d _targetTagPose;

  private int _tagId;

  public AlignWithBlockGridCmd(Drivetrain drive, XboxController controller,int targetTagID) {
    _drivetrain = drive;
    _controller = controller;

    addRequirements();
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {

    double rotationSpeed = 0;

    if (LimelightHelpers.getTV("")) {

      // Calculate angular turn power
      // -1.0 required to ensure positive PID controller effort _increases_ yaw
      var result = LimelightHelpers.getLatestResults("");
      var aprilTagTargets = result.targetingResults.targets_Fiducials;
      _targetTagPose = null;
      for (var tag :
          aprilTagTargets) {
        if (tag.fiducialID == 6) {
          _targetTagPose = tag.getRobotPose_TargetSpace();
        }

      }
      if (_targetTagPose != null) {
        rotationSpeed = -_turnController.calculate(_targetTagPose.getRotation().getZ(), 0);
      }


      _drivetrain.drive(_controller.getRightTriggerAxis(), _controller.getLeftTriggerAxis(),
          rotationSpeed, false, true);
    }

    //DriverStation.reportWarning("foundTarget: " + result.hasTargets() + " rotationSpeed: " + rotationSpeed, false);

  }

  @Override
  public void end(boolean interrupted) {
  }

  /**
   * Stops if any of the following are true:
   * <p>
   * if the robot is within a certain distance of the target<br>
   * if the controller is being used to turn<br>
   * if the robot loses the target
   * @return true when the command should end.
   */
  @Override
  public boolean isFinished() {
    return (_targetTagPose.getX() > -.1 && _targetTagPose.getX() < .1)
        || Math.abs(_controller.getRightX()) > .3 || !LimelightHelpers.getTV("");
  }

}
