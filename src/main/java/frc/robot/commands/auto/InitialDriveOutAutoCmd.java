package frc.robot.commands.auto;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utilities.RobotNav;

public class InitialDriveOutAutoCmd extends CommandBase {
  private final PIDController _controller = new PIDController(0.1, 0, 0);
private Drivetrain _drivetrain;
  private RobotNav _robotNav;
  private double _distanceToDrive;

  /**
   * Creates a new InitialDriveOutAutoCmd.
   */
  public InitialDriveOutAutoCmd(Drivetrain drivetrain, RobotNav robotNav,double distance){
    _drivetrain = drivetrain;
    _robotNav = robotNav;
    _distanceToDrive = distance;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }
  {

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _drivetrain.drive(0.5, 0, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return RobotNav.getGyro().getDisplacementX() >= _distanceToDrive;
  }

}
