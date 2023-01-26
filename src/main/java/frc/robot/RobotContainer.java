// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ControllerDriveCmd;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
  private XboxController driveController = new XboxController(0);
  public RobotContainer()
  {

    // Configure the trigger bindings
    configureBindings();
  }


  /** Use this method to define your trigger->command mappings. */
  private void configureBindings()
  {
  /*  new Trigger(() -> (driveController.getRightTriggerAxis() != 0)
        || driveController.getLeftTriggerAxis() != 0 || driveController.getRightX() != 0).onTrue(
        new ControllerDriveCmd(driveController.getRightTriggerAxis(),
            driveController.getLeftTriggerAxis(),
            driveController.getRightX()));*/
  }



  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return Auto.exampleAuto(exampleSubsystem);
  }
}
