package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utilities.RobotNav;

public class Turn180 extends CommandBase {
  private static Drivetrain _driveTrain = new Drivetrain();
  private long _cTime;
  private final double currentYaw;
  /**
   * Command that finds a path from the robot's current position to the target position and sends it to the drive train.
   */
  public Turn180() {

    currentYaw = RobotNav.getGyro().getYaw();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    _driveTrain.drive(0,0,0.5,false,false);

    DriverStation.reportError(RobotNav.getGyro().getYaw()-currentYaw +"",false);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _driveTrain.drive(0,0,0,false,false);
    Command gofor = new DriveForwardXCmd();
    gofor.schedule();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    var yaw = Math.abs(RobotNav.getGyro().getYaw()-currentYaw );
    if( yaw < 180 && yaw > 177){
      return true;
    }else{
      return false;
    }
  }

}
