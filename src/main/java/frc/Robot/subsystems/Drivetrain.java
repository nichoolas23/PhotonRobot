package frc.Robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.controller.DifferentialDriveWheelVoltages;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BiConsumer;

public class Drivetrain extends SubsystemBase {


  private final WPI_TalonSRX[] wpi_talonSRXES = new WPI_TalonSRX[]{new WPI_TalonSRX(1),
      new WPI_TalonSRX(3), new WPI_TalonSRX(2), new WPI_TalonSRX(4)};
  private final MotorControllerGroup leftDrive = new MotorControllerGroup(wpi_talonSRXES[0], wpi_talonSRXES[1]);
  private final MotorControllerGroup rightDrive = new MotorControllerGroup(wpi_talonSRXES[2], wpi_talonSRXES[3]);

  public MotorControllerGroup getLeftDrive() {return leftDrive;}

  public MotorControllerGroup getRightDrive() {return rightDrive;}



  private static final  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(.5);

  private static DifferentialDriveWheelSpeeds speeds = new DifferentialDriveWheelSpeeds(0, 0);
  private static DifferentialDriveWheelVoltages voltages = new DifferentialDriveWheelVoltages(0, 0);




  final double LINEAR_P = 0.1;
  final double LINEAR_D = 0.0;
  PIDController forwardController = new PIDController(LINEAR_P, 0, LINEAR_D);

  final double ANGULAR_P = 0.1;
  final double ANGULAR_D = 0.0;
  PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);

  public static DifferentialDriveKinematics getKinematics() {
    return kinematics;
  }

  public static DifferentialDriveWheelSpeeds getSpeeds() {
    return speeds;
  }

  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    this.speeds = speeds;
  }

  public static BiConsumer<Double,Double> getVoltages() {
    return ((aDouble, aDouble2) -> {
     voltages.left = aDouble;
      voltages.right = aDouble2;
    });
  }

  public void setVoltages(DifferentialDriveWheelVoltages voltages) {
    this.voltages = voltages;
  }

}
