package frc.robot.utilities;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class RobotNav {

  private static AHRS _gyro = new AHRS();
  private static double _rotateVal;


  //Limelight data
  private static NetworkTable _table =  NetworkTableInstance.getDefault().getTable("limelight");

  private static boolean _hasTarget;
  private static long _pipelineLatency;
  private static long _camLatency;

  private static double[] _globalBotPose;

  private static double[] _redBotPose; // orgin set to red driverstation

  private static double[] _blueBotPose;

  private static double[] _camPoseFromTarget;

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
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    _hasTarget = _table.getEntry("tv").getInteger(0) == 1;
    _pipelineLatency = _table.getEntry("tl").getInteger(0);
    _camLatency =_pipelineLatency +11;

    _globalBotPose = _table.getEntry("botpose").getDoubleArray(new double[6]);
    _blueBotPose = _table.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
    _redBotPose = _table.getEntry("botpose_wpired").getDoubleArray(new double[6]);
    _camPoseFromTarget = _table.getEntry("camerapose_targetspace").getDoubleArray(new double[6]);


//read values periodically
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);

//post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);

    SmartDashboard.putNumber("LimelightLatency", _camLatency);
    SmartDashboard.putBoolean("LimelightHasTarget", _hasTarget);
    SmartDashboard.putNumber("LimelightPipelineLatency", _pipelineLatency);
    SmartDashboard.putNumberArray("LimelightGlobalBotPose", _globalBotPose);
    SmartDashboard.putNumberArray("LimelightBlueBotPose", _blueBotPose);
    SmartDashboard.putNumberArray("LimelightRedBotPose", _redBotPose);
    SmartDashboard.putNumberArray("LimelightCamPoseFromTarget", _camPoseFromTarget);

  }
}

