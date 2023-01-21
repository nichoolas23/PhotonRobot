package frc.Robot.AutoDrive;

import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import frc.Robot.Robot;
import frc.Robot.RobotSystems.Drivetrain;
import frc.Robot.RobotSystems.RobotNav;
import org.photonvision.PhotonCamera;


public class AutoMode  {
  public static void autoDriver(RobotNav robotNavData){
      // Do any initial stuff here

    executeAuto(robotNavData);

  }


  private static void executeAuto(RobotNav robotNavData)  {
    var yaw = robotNavData.getGyro().getYaw();
   //TODO: (for lola) determine how to align the robot between the two apriltags
    // Try using Robot.photonTrackedTargets which is a list of all the tracked vision targets use the PathPlanner class to create a path to desired point











    PathPlanner planner = new PathPlanner();
    //TODO: (for nick later) implement A* algorithm taking current pos and end point and puts all points into path planner




  }





}
