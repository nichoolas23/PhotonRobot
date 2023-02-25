package frc.robot.commands.auto;


import static frc.robot.Constants.RobotConstants.ControlsConstants.ALIGNMENT;
import static frc.robot.PhysicalInputs.XBOX_CONTROLLER;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utilities.LimelightHelpers;
import frc.robot.utilities.RobotNav;


public class AlignWithBlockGridCmd extends CommandBase {


  private final XboxController _controller;

  private Drivetrain _drivetrain;
  private static double _targetTagHeadingError;
  private static double _toCorrect;
  private int _tagId;

  public AlignWithBlockGridCmd(Drivetrain drive, XboxController controller, int targetTagID) {
    _drivetrain = drive;
    _controller = controller;
    _tagId = targetTagID;
    addRequirements();
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {

    double rotationSpeed = 0;
    Pose3d targetTag;
    if (LimelightHelpers.getTV("")) {

      // Calculate angular turn power
      // -1.0 required to ensure positive PID controller effort _increases_ yaw
      var result = LimelightHelpers.getLatestResults("");
      var aprilTagTargets = result.targetingResults.targets_Fiducials;
      for (var tag :
          aprilTagTargets) {
        if (tag.fiducialID == 6) {
          _targetTagHeadingError = tag.tx;

        }
      }

      _toCorrect = RobotNav.getHeading() + _targetTagHeadingError;
      ALIGNMENT.setGoal(_toCorrect);
      if (_targetTagHeadingError != 0.02) {
        rotationSpeed = ALIGNMENT.getController().calculate(RobotNav.getHeading(), _toCorrect);
      }
      SmartDashboard.putNumber("Period", ALIGNMENT.getController().getPeriod());
      SmartDashboard.putNumber("Alignment measurementZ", _targetTagHeadingError);
      SmartDashboard.putNumber("Heading", RobotNav.getHeading());
      SmartDashboard.putNumber("Corrected Heading", _toCorrect);

      if (Math.abs(rotationSpeed) < .4) {
        if (rotationSpeed < 0) {
          rotationSpeed -= 0.4;
        } else if (rotationSpeed > 0) {
          rotationSpeed += .4;
        }
      }
      // period = .02

      SmartDashboard.putNumber("Alignment Rot Speed", rotationSpeed);
      _drivetrain.drive(XBOX_CONTROLLER.getRightTriggerAxis(), XBOX_CONTROLLER.getLeftTriggerAxis(),
          rotationSpeed);
    }


  }

  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("Stabilized", true);
  }

  /**
   * Stops if any of the following are true:
   * <p>
   * if the robot is within a certain distance of the target<br> if the controller is being used to
   * turn<br> if the robot loses the target
   *
   * @return true when the command should end.
   */
  @Override
  public boolean isFinished() {
    return (
        Math.abs(_controller.getRightX()) > .3 || (RobotNav.getGyro().getRate() < .1
            && ALIGNMENT.getController().atSetpoint()));
  }

}
