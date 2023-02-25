// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



import static frc.robot.Constants.RobotConstants.STAB_PID_D;
import static frc.robot.Constants.RobotConstants.STAB_PID_I;
import static frc.robot.Constants.RobotConstants.STAB_PID_P;
import static frc.robot.PhysicalInputs.XBOX_CONTROLLER;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ControllerDriveCmd;

import frc.robot.commands.StabilizedDriveCmd;
import frc.robot.commands.auto.AlignWithBlockGridCmd;
import frc.robot.commands.auto.Auto;

import frc.robot.commands.auto.TurnToAngleProfiled;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.RobotAlignment;
import frc.robot.utilities.RobotNav;
import frc.robot.utilities.TrajectoryGen;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
@SuppressWarnings("ClassNamePrefixedWithPackageName")
public class RobotContainer {

  private final RobotNav _robotNav = new RobotNav();
  private Drivetrain _drivetrain = new Drivetrain();
  private final XboxController _driveController = new XboxController(0);


  public RobotContainer() {

    // Configure the trigger bindings
    configureBindings();
  }


  /**
   * Use this method to define your trigger->command mappings.
   */
  private void configureBindings() {
    new Trigger(()->(XBOX_CONTROLLER.getRightX() > -.06 && XBOX_CONTROLLER.getRightX() <.06) && Math.abs(RobotNav.getGyro().getRate())<4)
        .whileTrue(new StabilizedDriveCmd(_drivetrain,XBOX_CONTROLLER,_robotNav))
        .whileFalse(new ControllerDriveCmd(_drivetrain,XBOX_CONTROLLER));
    new Trigger(_driveController::getAButtonPressed)
        .onTrue(new AlignWithBlockGridCmd(_drivetrain,_driveController,6));

  }


  public Command getTeleopCommand() {


    return new ControllerDriveCmd(new Drivetrain(), _driveController);
  }
  public Command getAutonomousCommand(){
    return Auto.autoFactory(_drivetrain);
  }







  private static Pose2d getbotpose(){
    return RobotNav.get_diffDrivePose().getEstimatedPosition();}
}
