package frc.Robot.DriveModes.AutoDrive;


import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.Robot.Constants.FieldConstants;
import frc.Robot.subsystems.Drivetrain;
import frc.Robot.subsystems.RobotNav;
import java.util.List;


public class AutoPathfinder extends FieldConstants {

   private static final double KS = 0.22;
   private static final double KV = 0.22;
   private static final double KA = 0.22;

   public static void findPath(){

  // setup bounding box for charging station
  //


   var trajectory = TrajectoryGenerator.generateTrajectory(List.of(RobotNav.getRobotPose2d(), blueChargingStation),
       new TrajectoryConfig(1, 1));


}
   public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
      return new SequentialCommandGroup(
          new InstantCommand(() -> {
             // Reset odometry for the first path you run during auto
             if(isFirstPath){

                this.resetOdometry(traj.getInitialPose());
             }
          }),
          new PPRamseteCommand(
              traj,
              RobotNav::getRobotPose2d, // Pose supplier
              new RamseteController(),
              new SimpleMotorFeedforward(KS, KV, KA),
              Drivetrain.getKinematics(), // DifferentialDriveKinematics
              Drivetrain::getSpeeds, // DifferentialDriveWheelSpeeds supplier
              new PIDController(0, 0, 0), // Left controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
              new PIDController(0, 0, 0), // Right controller (usually the same values as left controller)
              Drivetrain.getVoltages(), // Voltage biconsumer
              true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
               this // Requires this drive subsystem
          )
      );
   }
}
