package frc.robot.commands.auto;

import static frc.robot.Constants.VisionConstants.PHOTON_CAMERA;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import org.photonvision.PhotonCamera;

public class AimAtTargetCmd extends CommandBase {
  private boolean run = true;
  final double ANGULAR_P = 0.2;
  final double ANGULAR_D = 0.0;

  private final XboxController _controller = new XboxController(0);
  PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);
  PhotonCamera camera = new PhotonCamera("photoncamera");
Drivetrain _drivetrain = new Drivetrain();
  public AimAtTargetCmd(){

    addRequirements();
  }

  @Override
  public void initialize(){
  }
  @Override
  public void execute(){
    if (Math.abs(_controller.getRightX()) > .09){ // if manual rotation is attempted this stops
      run = false;
    }
    double rotationSpeed;
    var result = camera.getLatestResult();
    if (result.hasTargets()) {
      DriverStation.reportError("HAS TARGERTS",false);
      // Calculate angular turn power
      // -1.0 required to ensure positive PID controller effort _increases_ yaw
      rotationSpeed = -turnController.calculate(result.getBestTarget().getYaw(), 0);

      DriverStation.reportError(rotationSpeed + "",false);
      _drivetrain.drive(_controller.getRightTriggerAxis(),_controller.getLeftTriggerAxis(),rotationSpeed,false,true);
    } else {
      // If we have no targets, stay still.
      run = false;
    }
    //DriverStation.reportWarning("foundTarget: " + result.hasTargets() + " rotationSpeed: " + rotationSpeed, false);

  }
  @Override
  public void end(boolean interrupted){
  }
  @Override
  public boolean isFinished(){
    return run;
  }

}
