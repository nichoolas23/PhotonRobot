package frc.robot.commands;

import static frc.robot.Constants.RobotConstants.LEFT_VOLTS_MAX;
import static frc.robot.Constants.RobotConstants.LEFT_VOLTS_SECONDS_PER_METER;
import static frc.robot.Constants.RobotConstants.LEFT_VOLTS_SECONDS_SQ_PER_METER;
import static frc.robot.Constants.RobotConstants.STAB_PID_D;
import static frc.robot.Constants.RobotConstants.STAB_PID_I;
import static frc.robot.Constants.RobotConstants.STAB_PID_P;
import static frc.robot.PhysicalInputs.XBOX_CONTROLLER;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utilities.RobotNav;

public class StabilizedDriveCmd extends ProfiledPIDCommand { // TODO: Does this really even need to be profiled (probably)

  /**
   * Creates a new StabilizedDriveCmd.
   */
  private Drivetrain _drivetrain;
  private XboxController _controller;
  private final SimpleMotorFeedforward _leftFeedforward =
      new SimpleMotorFeedforward(
          LEFT_VOLTS_MAX, LEFT_VOLTS_SECONDS_PER_METER, LEFT_VOLTS_SECONDS_SQ_PER_METER);
  private final SimpleMotorFeedforward _rightFeedforward = new SimpleMotorFeedforward(
      LEFT_VOLTS_MAX, LEFT_VOLTS_SECONDS_PER_METER, LEFT_VOLTS_SECONDS_SQ_PER_METER);

  //TODO: Tune this PID
  public StabilizedDriveCmd(Drivetrain drivetrain, XboxController controller,RobotNav robotNav) {
    super(
          new ProfiledPIDController(
              STAB_PID_P,
              STAB_PID_I,
              STAB_PID_D,new Constraints(.2,.1)),
          // Close the loop on the turn rate
        robotNav::getTurnRate,
        // Setpoint is 0
        0,
        // Pipe the output to the turning controls
        ( output,setpoint) ->
            drivetrain.drive(controller.getRightTriggerAxis(),controller.getLeftTriggerAxis(), output),

        // Require the robot drive
        drivetrain);

    SmartDashboard.putData("Stab pid",this.getController());

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements();

  }

  // Called when the command is initially scheduled.

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //DriverStation.reportError((_controller.getRightX() != 0) +"",true);
    return false;
  }


}
