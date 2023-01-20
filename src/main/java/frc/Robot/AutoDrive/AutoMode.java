package frc.Robot.AutoDrive;

import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import frc.Robot.RobotSystems.Drivetrain;
import frc.Robot.RobotSystems.RobotNav;


public class AutoMode  {
  public static void autoDriver(RobotNav robotNavData){
      // Do any initial stuff here

    executeAuto(robotNavData);

  }


  private static void executeAuto(RobotNav robotNavData)  {
    var yaw = robotNavData.getGyro().getYaw();
    PathPlanner planner = new PathPlanner();
    //TODO: implements A* algorithm taking current pos and end point and puts all points into path planner




  }





}
