package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants.PnuematicsConstants.PistonSelect;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Set;
import javax.print.DocFlavor.INPUT_STREAM;

public class Pnuematics extends SubsystemBase {



  public Pnuematics(int[] channels,double[] pulseDuration,boolean[] isExtend) {
    // Use addRequirements() here to declare subsystem dependencies.
    for (int i = 0; i < channels.length; i++) {
      _activePistons.add(new Solenoid(PneumaticsModuleType.CTREPCM, channels[i]));

    }




  }

  @Override
  public void periodic() {


    // This method will be called once per scheduler run
  }

  public void set_piston() {
    // This method will be called once per scheduler run during simulation
  }
}
