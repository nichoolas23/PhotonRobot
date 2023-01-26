package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pnuematics extends SubsystemBase {

  private Solenoid _piston = new Solenoid(PneumaticsModuleType.CTREPCM, 0);

  public Pnuematics() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  @Override
  public void periodic() {


    // This method will be called once per scheduler run
  }
}
