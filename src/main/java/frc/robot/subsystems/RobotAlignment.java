package frc.robot.subsystems;

import static frc.robot.Constants.RobotConstants.LEFT_VOLTS_MAX;
import static frc.robot.Constants.RobotConstants.LEFT_VOLTS_SECONDS_PER_METER;
import static frc.robot.Constants.RobotConstants.LEFT_VOLTS_SECONDS_SQ_PER_METER;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

public class RobotAlignment extends PIDSubsystem { //TODO: SIMPLIFY THE PID CONTROLLERS INTO THEIR OWN PACKAGES AND CLASSES this is disgusting

  private final SimpleMotorFeedforward _leftFeedforward =
      new SimpleMotorFeedforward(
          LEFT_VOLTS_MAX, LEFT_VOLTS_SECONDS_PER_METER, LEFT_VOLTS_SECONDS_SQ_PER_METER);
  private final SimpleMotorFeedforward _rightFeedforward = new SimpleMotorFeedforward(
      LEFT_VOLTS_MAX, LEFT_VOLTS_SECONDS_PER_METER, LEFT_VOLTS_SECONDS_SQ_PER_METER);

  public RobotAlignment(PIDController controller,
      double initialPosition) {
    super(controller, initialPosition);
    this.getController().setTolerance(.1);
    this.getController().enableContinuousInput(0, 360);

  }




  @Override
  protected void useOutput(double output, double setpoint) {

  }

  @Override
  protected double getMeasurement() {
    return 0;
  }
}
