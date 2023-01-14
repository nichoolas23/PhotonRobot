package frc.Field;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RoboField {

  private static final Field2d field = new Field2d();
  public static void fieldSetup(){
    SmartDashboard.putData("Field",field);
  }
  public static Pose2d fieldUpdate(Pose2d robotPos){
    field.setRobotPose(robotPos);
    return robotPos;
  }
}
