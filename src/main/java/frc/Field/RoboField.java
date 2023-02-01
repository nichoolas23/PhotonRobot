package frc.Field;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RoboField {

  private static final Field2d _field = new Field2d();

  public static void fieldSetup() {
    SmartDashboard.putData("Field", _field);
  }

  public static Pose2d fieldUpdate(Pose2d robotPos) {
    if(robotPos != null){
      _field.setRobotPose(robotPos);
      return robotPos;
    }else{
      return robotPos;
    }

  }
}
