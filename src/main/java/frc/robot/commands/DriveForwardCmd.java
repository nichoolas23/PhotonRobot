package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveForwardCmd extends CommandBase {
Drivetrain _drivetrain;
  public DriveForwardCmd(Drivetrain drive) {
    _drivetrain = drive;
    addRequirements(_drivetrain);

  }

  @Override
  public void initialize() {
  }


  @Override
  public void execute() {

      _drivetrain.drive(1,0,.2);


  }


  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return false;
  }}

