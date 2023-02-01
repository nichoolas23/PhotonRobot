package frc.robot.commands;

import static frc.robot.Constants.VisionConstants.PHOTON_CAMERA;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utilities.RobotNav;

public class ControllerDriveCmd extends CommandBase {


  private Drivetrain _drivetrain = new Drivetrain();

  private final XboxController _controller;

  public ControllerDriveCmd(Drivetrain drive, XboxController controller) {
    _drivetrain = drive;
    _controller = controller;
    addRequirements(_drivetrain);
  }


  @Override
  public void initialize() {
  }


  @Override
  public void execute() {
    _drivetrain.drive(_controller.getRightTriggerAxis(),_controller.getLeftTriggerAxis(), _controller.getRightX(),false);
    if(PHOTON_CAMERA.getLatestResult().hasTargets()){
      SmartDashboard.putNumber("TargetYaw", PHOTON_CAMERA.getLatestResult().getBestTarget().getYaw());
      SmartDashboard.putNumber("TargetPitch", PHOTON_CAMERA.getLatestResult().getBestTarget().getPitch());
      SmartDashboard.putNumber("TargetArea", PHOTON_CAMERA.getLatestResult().getBestTarget().getArea());
      SmartDashboard.putNumber("TargetSkew", PHOTON_CAMERA.getLatestResult().getBestTarget().getSkew());

    }
  }


  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
 }
