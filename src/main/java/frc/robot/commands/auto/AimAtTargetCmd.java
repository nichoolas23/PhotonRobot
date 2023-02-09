/*
package frc.robot.commands.auto;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;


public class AimAtTargetCmd extends CommandBase {
  private boolean run = true;
  final double ANGULAR_P = 0.1;
  final double ANGULAR_D = 0.045;

  private final XboxController _controller = new XboxController(0);
  PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);

  Drivetrain _drivetrain = new Drivetrain();
  public AimAtTargetCmd(){

    addRequirements();
  }

  @Override
  public void initialize(){
  }
  @Override
  public void execute(){
    if (Math.abs(_controller.getRightX()) > .3){ // if manual rotation is attempted this stops
      run = false;
    }
    double rotationSpeed;

    if (result.hasTargets()) {
      DriverStation.reportError("HAS TARGETS",false);
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

}*/
