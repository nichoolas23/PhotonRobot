package frc.robot.utilities;

import frc.robot.subsystems.Drivetrain;

public class EmergencyStopper {
  public static boolean isEmergencyStopped() {

    if (RobotNav.getGyro().getVelocityX() > 1.0 || RobotNav.getGyro().getVelocityY() > 1.0
        || RobotNav.getGyro().getVelocityZ() > 1.0 || RobotNav.getLeftEncoderVelocity() > 1.0 || RobotNav.getRightEncoderVelocity() > 1.0) {
      Drivetrain drivetrain = new Drivetrain();
      drivetrain.emergencyStop();
    }
    return false;
  }

}
