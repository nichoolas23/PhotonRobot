package frc.robot.commands;

import static frc.robot.Constants.RobotConstants.STAB_PID_D;
import static frc.robot.Constants.RobotConstants.STAB_PID_I;
import static frc.robot.Constants.RobotConstants.STAB_PID_P;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utilities.RobotNav;

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
