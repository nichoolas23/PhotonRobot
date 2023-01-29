// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.Field.RoboField;
import frc.robot.subsystems.RobotNav;
import java.util.ArrayList;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * The VM is configured to automatically run this class, and to call the methods corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {

  private Command _autonomousCommand;
  private Command _teleopCommand;


  private RobotContainer _robotContainer;
  public static List<PhotonTrackedTarget> photonTrackedTargets = new ArrayList<>();
  //public static PhotonCamera camera = new PhotonCamera("photonCam");

  /**
   * This method is run when the robot is first started up and should be used for any initialization
   * code.
   */


  @Override
  public void robotInit() {
    PathPlannerServer.startServer(5811);
    RoboField.fieldSetup();
    RobotNav.navInit();
    CommandScheduler.getInstance().run();
    _robotContainer = new RobotContainer();

  }

  @Override
  public void teleopInit() {
    _teleopCommand = _robotContainer.getTeleopCommand();
    if (_autonomousCommand != null)
    {
      _autonomousCommand.cancel();
      _teleopCommand.execute();

    }
  }

  @Override
  public void autonomousInit() {
    //_autonomousCommand = _robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (_autonomousCommand != null)
    {
      _autonomousCommand.schedule();

    }

  }

  @Override
  public void robotPeriodic() {


      /*if(camera.getLatestResult().hasTargets()){
       photonTrackedTargets = camera.getLatestResult().targets;
      }*/
  }




}
