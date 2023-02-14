// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import static frc.robot.Constants.RobotConstants.AUTO_MAX_ACCEL;
import static frc.robot.Constants.RobotConstants.AUTO_MAX_SPEED;
import static frc.robot.Constants.RobotConstants.DRIVE_KINEMATICS;
import static frc.robot.Constants.RobotConstants.P_GAIN_DRIVE_VEL;
import static frc.robot.Constants.RobotConstants.RAMSETE_B;
import static frc.robot.Constants.RobotConstants.RAMSETE_ZETA;
import static frc.robot.Constants.RobotConstants.VOLTS_MAX;
import static frc.robot.Constants.RobotConstants.VOLTS_SECONDS_PER_METER;
import static frc.robot.Constants.RobotConstants.VOLTS_SECONDS_SQ_PER_METER;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.Field.RoboField;
import frc.robot.Constants.FieldConstants;
import frc.robot.commands.ControllerDriveCmd;

import frc.robot.commands.auto.AimAtTargetCmd;
import frc.robot.commands.auto.DriveForwardXCmd;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utilities.LimelightHelpers;
import frc.robot.utilities.RobotNav;
import java.lang.reflect.Field;
import java.util.List;

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

    new Trigger(_driveController::getAButtonPressed).onTrue(new AimAtTargetCmd());
    //new Trigger(_driveController::getXButtonPressed).onTrue(getAutonomousCommand());
    /*new Trigger(() -> _driveController.getRightX() != 0).onTrue()*/
  }

  public Command getTeleopCommand() {

    return new ControllerDriveCmd(new Drivetrain(), _driveController);
  }


  public Command getAutonomousCommand() {
    new Drivetrain();
    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                VOLTS_MAX,
                VOLTS_SECONDS_PER_METER,
                VOLTS_SECONDS_SQ_PER_METER),
            DRIVE_KINEMATICS,
            10);

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
            AUTO_MAX_SPEED,
            AUTO_MAX_ACCEL)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DRIVE_KINEMATICS)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow.  All units in meters.
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        RobotNav.get_diffDrivePose().getEstimatedPosition(), List.of(),
        FieldConstants.FIRST_BLUE_GRID,
        config
    );
    RoboField.putTraj(trajectory);

    RamseteCommand ramseteCommand =
        new RamseteCommand(
            trajectory,
            RobotContainer::getbotpose,
            new RamseteController(RAMSETE_B, RAMSETE_ZETA),
            new SimpleMotorFeedforward(
                VOLTS_MAX,
                VOLTS_SECONDS_PER_METER,
                VOLTS_SECONDS_SQ_PER_METER),
            DRIVE_KINEMATICS,
            _drivetrain::getWheelSpeeds,
            new PIDController(P_GAIN_DRIVE_VEL, 0, 0),
            new PIDController(P_GAIN_DRIVE_VEL, 0, 0),
            // RamseteCommand passes volts to the callback
            _drivetrain::setVoltages,
            _drivetrain);

    // Reset odometry to the starting pose of the trajectory.
    _drivetrain.resetOdometry(trajectory.getInitialPose());
    RoboField.putTraj(trajectory);
    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> _drivetrain.setVoltages(0, 0));
  }
  private static Pose2d getbotpose(){
    return RobotNav.get_diffDrivePose().getEstimatedPosition();}
}
