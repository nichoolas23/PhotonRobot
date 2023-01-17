package frc.Robot.RobotSystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

public class Drivetrain {
  private final WPI_TalonSRX[] wpi_talonSRXES = new WPI_TalonSRX[]{new WPI_TalonSRX(1),
      new WPI_TalonSRX(3), new WPI_TalonSRX(2), new WPI_TalonSRX(4)};
  private final MotorControllerGroup leftDrive = new MotorControllerGroup(wpi_talonSRXES[0], wpi_talonSRXES[1]);
  private final MotorControllerGroup rightDrive = new MotorControllerGroup(wpi_talonSRXES[2], wpi_talonSRXES[3]);

  public MotorControllerGroup getLeftDrive() {
    return leftDrive;
  }

  public MotorControllerGroup getRightDrive() {
    return rightDrive;
  }
}
