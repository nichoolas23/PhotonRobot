package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class ControllerDriveCmd extends CommandBase {


  private final Drivetrain _drivetrain;

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

      _drivetrain.drive(_controller.getRightTriggerAxis(),_controller.getLeftTriggerAxis(), _controller.getRightX());

  }


  @Override
  public void end(boolean interrupted) {
  }
  @Override
  public boolean isFinished() {
    return false;
  }

 }
