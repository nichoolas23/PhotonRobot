package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import java.sql.Driver;
import java.sql.Time;
import java.util.Timer;

public class DriveForwardXCmd extends CommandBase {

  private static Drivetrain _driveTrain = new Drivetrain();
  private long _cTime;

  /**
   * Command that finds a path from the robot's current position to the target position and sends it to the drive train.
   */
  public DriveForwardXCmd() {
    _cTime = System.currentTimeMillis();

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

          _driveTrain.drive(.5,0,0,false,false);



  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _driveTrain.drive(0,0,0,false,false);
    Command turn = new Turn180();
    turn.schedule();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(System.currentTimeMillis()-_cTime >= 3000){

      return true;
    }else{
      return false;
    }
  }

}
