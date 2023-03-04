package frc.robot.utilities;
//TODO: Maybe needed for competition
public class RobotConfigSettings {
  public enum EAutoSpeed {
    CRAWL(0.2),
    SLOW(0.4),
    MEDIUM(0.6),
    FAST(0.8),
    MAX(1.0);

    EAutoSpeed(double v) {
    }
  }

}
