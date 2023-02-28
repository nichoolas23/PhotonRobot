package frc.robot.commands.auto;

import static frc.robot.Constants.RobotConstants.TURN_ACCEL_DEG_PER_SECSQ_MAX;
import static frc.robot.Constants.RobotConstants.TURN_DEG_PER_SEC_MAX;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;


import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.RobotAlignment;
import frc.robot.utilities.RobotNav;

/** A command that will turn the robot to the specified angle using a motion profile. */
public class TurnToAngleProfiled extends ProfiledPIDCommand {
  /**
   * Turns to robot to the specified angle using a motion profile.
   *
   * @param targetAngleDegrees The angle to turn to
   * @param drive The drive subsystem to use
   */
private final ShuffleboardTab tab = Shuffleboard.getTab("Heading");

public TurnToAngleProfiled(double targetAngleDegrees, Drivetrain drive, RobotAlignment robotAlignment) {

    super(
        new ProfiledPIDController(
            robotAlignment.getController().getP(),
            robotAlignment.getController().getI(),
            robotAlignment.getController().getD(),
            new TrapezoidProfile.Constraints(
               TURN_DEG_PER_SEC_MAX,
                TURN_ACCEL_DEG_PER_SECSQ_MAX)),
        // Close loop on heading
        RobotNav::getHeading,
        // Set reference to target
        targetAngleDegrees,
        // Pipe output to turn robot
        (output, setpoint) -> drive.drive(0, output),
        // Require the drive
        drive);

    // Set the controller to be continuous (because it is an angle controller)
    getController().enableContinuousInput(-180, 180);
    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    getController()
        .setTolerance(.1, .2);
  }

  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    return getController().atGoal();
  }
}
