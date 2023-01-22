package frc.Robot.DriveModes.AutoDrive;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import frc.Robot.RobotSystems.RobotNav;


public class AutoMode extends TimedRobot {
  private final Timer timer = new Timer();
  public static void autoDriver(RobotNav robotNavData){
      // Do any initial stuff here

    executeAuto(robotNavData);

  }
  @Override
  public void autonomousInit() {
    timer.reset();
    timer.start();

  }
  @Override
  public void autonomousPeriodic() {
    RobotNav.updatePose();
  }


  private static void executeAuto(RobotNav robotNavData)  {
    var yaw = robotNavData.getGyro().getYaw();
   //TODO: (for lola) determine how to align the robot between the two apriltags
    // Try using Robot.photonTrackedTargets which is a list of all the tracked vision targets use the PathPlanner class to create a path to desired point


    PathPlanner planner = new PathPlanner();
    PathPlannerTrajectory pathPlannerTrajectory = new PathPlannerTrajectory();












    //PathPlanner planner = new PathPlanner();
    //TODO: (for nick later) implement A* algorithm taking current pos and end point and puts all points into path planner




  }





}
