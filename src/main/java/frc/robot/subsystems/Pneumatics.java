package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants.PneumaticsConstants.RPistonControl;

public class Pneumatics extends SubsystemBase {


  public Pneumatics() {

  }

  /**
   * Sets the piston to the desired state.
   *
   * @param piston
   */
  public void setPiston(RPistonControl piston) { //TODO: Get rid of the damn record this is stupid
    piston.solenoid().setPulseDuration(piston.pulseDuration());
    try {
      Thread.sleep(piston.delay() * 1000);
    } catch (InterruptedException e) {
      throw new RuntimeException(e);
    }
    piston.solenoid().startPulse();
  }

  @Override
  public void periodic() {
  }



  public boolean setPiston( boolean isEnabled, Solenoid... solenoidArray) {
    for (int i = 0; i < solenoidArray.length; i++) {
      solenoidArray[i].toggle();

    }
return true;
  }
}
