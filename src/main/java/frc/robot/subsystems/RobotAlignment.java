package frc.robot.subsystems;

import static frc.robot.Constants.RobotConstants.VOLTS_MAX;
import static frc.robot.Constants.RobotConstants.VOLTS_SECONDS_PER_METER;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

public class RobotAlignment extends ProfiledPIDSubsystem {
  private final SimpleMotorFeedforward _shooterFeedforward =
      new SimpleMotorFeedforward(
          VOLTS_MAX, VOLTS_SECONDS_PER_METER);
  public RobotAlignment(ProfiledPIDController controller,
      double initialPosition) {
    super(controller, initialPosition);
this.getController().setTolerance(.1);
this.getController().enableContinuousInput(0,360);

  }


  @Override
  protected void useOutput(double output, State setpoint) {

  }

  @Override
  protected double getMeasurement() {
    return 0;
  }
}
