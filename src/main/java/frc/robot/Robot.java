// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.RobotConstants.ControlsConstants.ALIGNMENT;

import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.Field.RoboField;
import frc.robot.commands.auto.Auto;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utilities.RobotNav;


/**
 * The VM is configured to automatically run this class, and to call the methods corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {


  private final Drivetrain _drivetrain = new Drivetrain();
  private final RobotNav _robotNav = new RobotNav();
  private Command _autonomousCommand;
  private RobotContainer _robotContainer;

  /**
   * This method is run when the robot is first started up and should be used for any initialization
   * code.
   */


  @Override
  public void robotInit() {

    PathPlannerServer.startServer(5811);
    RoboField.fieldSetup();
    _robotContainer = new RobotContainer();
    RobotNav.setStdDevVision();
    _drivetrain.setBrakeMode();
  }

  @Override
  public void teleopInit() {
    Command _teleopCommand = _robotContainer.getTeleopCommand();
    if (_autonomousCommand != null) {
      _autonomousCommand.cancel();

    }
    SmartDashboard.putData("alignment", ALIGNMENT.getController());
_teleopCommand.schedule();

  }

  @Override
  public void disabledInit() {
    CommandScheduler.getInstance().cancelAll();
  }


  @Override
  public void autonomousInit() {
    _autonomousCommand = _robotContainer.getAutonomousCommand();
    if (_autonomousCommand != null) {
      _autonomousCommand.schedule();

    }
  }

  @Override
  public void robotPeriodic() {
    _drivetrain.updateOdometry();
    _robotNav.updateLL();
    CommandScheduler.getInstance().run();
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.

    CommandScheduler.getInstance().cancelAll();
    ShuffleboardTab tab = Shuffleboard.getTab("Heading");
    GenericEntry alignmentEnable = tab.add("Heading Enable", false).getEntry();
    alignmentEnable.setBoolean(true);

  }

}
