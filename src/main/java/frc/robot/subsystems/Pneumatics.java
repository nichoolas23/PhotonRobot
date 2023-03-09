package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants.PneumaticsConstants.RPistonControl;

public class Pneumatics extends SubsystemBase {


  public Pneumatics() {

  }




  @Override
  public void periodic() {
  }



  public boolean setPiston( Solenoid... solenoidArray) {
    for (Solenoid solenoid : solenoidArray) {
      solenoid.toggle();

    }
return true;
  }
}
