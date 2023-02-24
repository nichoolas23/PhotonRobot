package frc.robot.subsystems;

import static frc.robot.Constants.RobotConstants.VOLTS_MAX;
import static frc.robot.Constants.RobotConstants.VOLTS_SECONDS_PER_METER;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.button.NetworkButton;

public class RobotAlignment extends PIDSubsystem {
  private final SimpleMotorFeedforward _shooterFeedforward =
      new SimpleMotorFeedforward(
          VOLTS_MAX, VOLTS_SECONDS_PER_METER);
  public RobotAlignment(PIDController controller,
      double initialPosition) {
    super(controller, initialPosition);


  }

  @Override
  protected void useOutput(double output, double setpoint) {

  }

  @Override
  protected double getMeasurement() {
    return 0;
  }
}
