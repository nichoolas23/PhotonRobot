// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



import static frc.robot.Constants.RobotConstants.PneumaticsConstants.WRIST_PISTON;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.RobotConstants.PneumaticsConstants.RPiston;
import frc.robot.commands.ControllerDriveCmd;
import frc.robot.commands.PistonExtendCmd;

import frc.robot.commands.auto.AimAtTargetCmd;
import frc.robot.commands.auto.PathFindCommand;
//import frc.robot.commands.auto.PathFollowCommand;
import frc.robot.subsystems.Drivetrain;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
@SuppressWarnings( "ClassNamePrefixedWithPackageName")
public class RobotContainer
{

  private final XboxController _driveController = new XboxController(0);
  public RobotContainer()
  {

    // Configure the trigger bindings
    configureBindings();
  }


  /** Use this method to define your trigger->command mappings. */
  private void configureBindings()
  {
    new Trigger(_driveController::getAButtonPressed).onTrue(new AimAtTargetCmd());

  }

  public Command getTeleopCommand(){

    return new ControllerDriveCmd(new Drivetrain(),_driveController);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  /*public Command getAutonomousCommand(){
    return new PathFollowCommand();
  }*/
}
