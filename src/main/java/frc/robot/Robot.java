// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.RobotConstants.LEFT_ENCODER;
import static frc.robot.Constants.RobotConstants.RIGHT_ENCODER;

import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.Field.RoboField;
import frc.robot.commands.auto.PathFindCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utilities.RobotNav;


/**
 * The VM is configured to automatically run this class, and to call the methods corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {

  XboxController controller = new XboxController(0);

  private final Drivetrain _drivetrain = new Drivetrain();
  private Command _autonomousCommand;
  private RobotContainer _robotContainer;
  private final RobotNav _robotNav = new RobotNav();

  /**
   * This method is run when the robot is first started up and should be used for any initialization
   * code.
   */


  @Override
  public void robotInit() {
    PathPlannerServer.startServer(5811);
    RoboField.fieldSetup();
    _robotContainer = new RobotContainer();

  }

  @Override
  public void teleopInit() {
    Command _teleopCommand = _robotContainer.getTeleopCommand();
    if (_autonomousCommand != null) {
      _autonomousCommand.cancel();

    }

    var command = new PathFindCommand();
    command.schedule();
    _teleopCommand.schedule();
  }

  @Override
  public void autonomousInit() {
   // _autonomousCommand = _robotContainer.getAutonomousCommand();
    // schedule the autonomous command (example)
    if (_autonomousCommand != null) {
      _autonomousCommand.schedule();

    }

  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    _drivetrain.updateOdometry();
    _robotNav.updateLL();
  /*  if (RobotNav.getEstimatedRobotPose() != null) {
      RoboField.fieldUpdate(RobotNav.getEstimatedRobotPose().estimatedPose.toPose2d());
    }*/
    if (controller.getBButton()) {
      LEFT_ENCODER.reset(); //-357.750000
      RIGHT_ENCODER.reset(); //355.000000
    }

  }
}
