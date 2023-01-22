package frc.Robot.DriveModes.TeleopDrive;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import frc.Robot.RobotSystems.Drivetrain;
import frc.Robot.RobotSystems.RobotNav;

public class TeleopMode extends TimedRobot {
  Drivetrain drivetrain = new Drivetrain();
  private XboxController m_controller = new XboxController(0);

  @Override
  public void teleopInit() {
    drivetrain.getLeftDrive().set(0);
    drivetrain.getRightDrive().set(0);
  }
  @Override
  public void teleopPeriodic(){

    if (m_controller.getAButtonPressed()) {
      drivetrain.getLeftDrive().stopMotor();
      drivetrain.getRightDrive().stopMotor();
    }

    double forward = m_controller.getLeftTriggerAxis() * .6;
    double reverse = m_controller.getRightTriggerAxis() * .6;
    RobotNav.setRotateVal(m_controller.getRightX() * .6);

    drivetrain.getLeftDrive().set(forward + RobotNav.getRotateVal());
    drivetrain.getRightDrive().set(forward - RobotNav.getRotateVal());

    drivetrain.getLeftDrive().set(-1.0 * reverse - RobotNav.getRotateVal());
    drivetrain.getRightDrive().set(-1.0 * reverse + RobotNav.getRotateVal());
  }
}
