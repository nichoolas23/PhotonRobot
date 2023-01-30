package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pneumatics extends SubsystemBase {



  public Pneumatics(int[] channels,double[] pulseDuration,boolean[] isExtend) {
    // Use addRequirements() here to declare subsystem dependencies.
    for (int i = 0; i < channels.length; i++) {
     // _activePistons.add(new Solenoid(PneumaticsModuleType.CTREPCM, channels[i]));

    }




  }

  @Override
  public void periodic() {
  }

  public void setPiston() {

  }
}
