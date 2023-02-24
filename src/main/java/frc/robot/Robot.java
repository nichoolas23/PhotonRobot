// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.RobotConstants.LEFT_ENCODER;
import static frc.robot.Constants.RobotConstants.RIGHT_ENCODER;
import static frc.robot.PhysicalInputs.XBOX_CONTROLLER;

import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.Field.RoboField;
import frc.robot.commands.StabilizedDriveCmd;
import frc.robot.commands.auto.PathFindCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.RobotAlignment;
import frc.robot.utilities.RobotNav;
import frc.robot.utilities.TrajectoryGen;


/**
 * The VM is configured to automatically run this class, and to call the methods corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {



  private final Drivetrain _drivetrain = new Drivetrain();
  private final RobotAlignment _robotAlignment = new RobotAlignment(new PIDController(0, 0, 0), 0);
  private Command _autonomousCommand;
  private RobotContainer _robotContainer;
  private final RobotNav _robotNav = new RobotNav();

  /**
   * This method is run when the robot is first started up and should be used for any initialization
   * code.
   */


  @Override
  public void robotInit() {
    SmartDashboard.putNumber("num",RobotNav.getGyro().getRate());
    PathPlannerServer.startServer(5811);

    RoboField.fieldSetup();
    _robotContainer = new RobotContainer();
    RobotNav.setStdDevVision();
    _drivetrain.setBrakeMode();
    SmartDashboard.putData("PID Controller",_robotAlignment.getController());
  }

  @Override
  public void teleopInit() {
    Command _teleopCommand = _robotContainer.getTeleopCommand();
    if (_autonomousCommand != null) {
      _autonomousCommand.cancel();

    }

    var command = new PathFindCommand(_drivetrain);
    //command.schedule();

  }
@Override
public void disabledInit()
{CommandScheduler.getInstance().cancelAll();}


  @Override
  public void autonomousInit() {
    Command _autonomousCommand;
    _autonomousCommand = TrajectoryGen.getTrajCmd(_drivetrain);
    // schedule the autonomous command (example)
    if (_autonomousCommand != null) {
      _autonomousCommand.schedule();


    }
  }

  @Override
  public void robotPeriodic() {
   SmartDashboard.putData("navx",RobotNav.getGyro());

    /*if(Math.abs(RobotNav.getGyro().getRate())>1){
      CommandScheduler.getInstance().cancelAll();
      CommandScheduler.getInstance().disable();
    }*/
    _drivetrain.updateOdometry();
    _robotNav.updateLL();
    CommandScheduler.getInstance().run();




  /*  if (RobotNav.getEstimatedRobotPose() != null) {
      RoboField.fieldUpdate(RobotNav.getEstimatedRobotPose().estimatedPose.toPose2d());
    }*/
    if (XBOX_CONTROLLER.getBButton()) {
      LEFT_ENCODER.reset(); //-357.750000
      RIGHT_ENCODER.reset(); //355.000000
    }

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
