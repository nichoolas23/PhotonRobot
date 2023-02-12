package frc.robot.utilities;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class RobotNav extends LimelightHelpers {

  private static AHRS _gyro = new AHRS();
  private static double _rotateVal;


  //Limelight data
  private static NetworkTable _table =  NetworkTableInstance.getDefault().getTable("limelight");

  private static boolean _hasTarget;




  /**
   * Initializes the robot navigation data
   * Updates all poses so they aren't null when called
   */
public RobotNav(){
  _table.getEntry("<variablename>").getDouble(0);
}

  public static AHRS getGyro() {
    return _gyro;
  }

  public static void setRotateVal(double _rotateVal) {
    RobotNav._rotateVal = _rotateVal;
  }


  public void updateLL(){
    LimelightHelpers.getLatestResults("limelight");
    _hasTarget = LimelightHelpers.getTV("limelight");
    if(_hasTarget){
      SmartDashboard.putNumber("Limelight X", LimelightHelpers.getTX("limelight"));
      SmartDashboard.putNumber("Limelight Y", LimelightHelpers.getTY("limelight"));
      SmartDashboard.putNumber("Limelight Area", LimelightHelpers.getTA("limelight"));
      SmartDashboard.putNumber("Limelight Short", LimelightHelpers.getFiducialID("limelight"));
    }
  }
}

