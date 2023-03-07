// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import static frc.robot.PhysicalInputs.ARM_LIMIT_SWITCH;
import static frc.robot.PhysicalInputs.XBOX_CONTROLLER;
import static frc.robot.commands.auto.Auto.autoFactory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.FieldConstants;
import frc.robot.commands.arm.ArmExtendCmd;
import frc.robot.commands.arm.ArmRaiseCmd;
import frc.robot.commands.ChangeGearCmd;
import frc.robot.commands.arm.ManualArmControlCmd;
import frc.robot.commands.claw.ClawIntakeCmd;
import frc.robot.commands.ControllerDriveCmd;
import frc.robot.commands.wrist.ManualWristRaiseCmd;
import frc.robot.commands.wrist.OpenWristCmd;
import frc.robot.commands.StabilizedDriveCmd;
import frc.robot.commands.wrist.WristRaiseCmd;
import frc.robot.commands.auto.AlignWithBlockGridCmd;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Wrist;
import frc.robot.utilities.RobotNav;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
@SuppressWarnings("ClassNamePrefixedWithPackageName")
public class RobotContainer {

  private final RobotNav _robotNav = new RobotNav();
  private final Drivetrain _drivetrain = new Drivetrain();
  private final XboxController _driveController = new XboxController(0);
  private final Pneumatics _pneumatics = new Pneumatics();
  private final SendableChooser<Command> _commandSendableChooser = new SendableChooser<>();
  private final Wrist _wrist = new Wrist();
private final Arm _arm = new Arm();

  public RobotContainer() {
    configureAutoChooser();

    SmartDashboard.putData(_commandSendableChooser);
    
    // Configure the trigger bindings
    configureBindings();
  }

  private void configureAutoChooser() {
    _commandSendableChooser.addOption("Blue Auto", autoFactory(_drivetrain, _robotNav,
        FieldConstants.BLUE_GRID_TOP_LEFT, true));
    _commandSendableChooser.addOption("Red Auto", autoFactory(_drivetrain, _robotNav,FieldConstants.RED_GRID_TOP_RIGHT, false));
  }


  /**
   * Use this method to define your trigger->command mappings.
   */
  private void configureBindings() {
    new Trigger(ARM_LIMIT_SWITCH::get).onTrue(new InstantCommand(()-> {
      Drivetrain.wpi_talonSRXES[6].getSensorCollection().setQuadraturePosition( 0,0);
      Drivetrain.wpi_talonSRXES[3].getSensorCollection().setQuadraturePosition( 0,0);
    }));


    new Trigger(() -> (XBOX_CONTROLLER.getRightX() > -.06 && XBOX_CONTROLLER.getRightX() < .06)
        && Math.abs(RobotNav.getGyro().getRate()) < 4)
        .whileTrue(new StabilizedDriveCmd(_drivetrain, XBOX_CONTROLLER, _robotNav))
        .whileFalse(new ControllerDriveCmd(_drivetrain, XBOX_CONTROLLER));
    new Trigger(_driveController::getAButtonPressed)
        .onTrue(new AlignWithBlockGridCmd(_drivetrain, _driveController, 6));
  /*  new Trigger(_driveController::getXButtonPressed)
        .onTrue(new );*/
    new Trigger(_driveController::getBButtonPressed)
        .onTrue(new ChangeGearCmd(_pneumatics));
    new Trigger(_driveController::getLeftBumperPressed).whileTrue(new OpenWristCmd(_pneumatics));
  new Trigger(_driveController::getYButtonPressed)
        .onTrue(new ArmRaiseCmd(_arm).andThen(new WristRaiseCmd(_wrist)));
    new Trigger(_driveController::getRightBumperPressed)
        .onTrue(new ClawIntakeCmd());
    new Trigger(_driveController::getLeftStickButtonPressed)
        .onTrue(new ArmExtendCmd(_pneumatics));

  }


  public Command getTeleopCommand() {

    return new ControllerDriveCmd(new Drivetrain(), _driveController).alongWith(new ManualArmControlCmd(_arm),
        new ManualWristRaiseCmd(_wrist));
  }

  public Command getAutonomousCommand() {
    return _commandSendableChooser.getSelected();
  }


  private static Pose2d getbotpose() {
    return RobotNav.get_diffDrivePose().getEstimatedPosition();
  }
}
