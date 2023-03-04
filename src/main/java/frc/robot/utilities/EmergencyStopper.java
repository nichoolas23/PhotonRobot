package frc.robot.utilities;

import frc.robot.subsystems.Drivetrain;

public class EmergencyStopper {

  //TODO: remove this maybe isn't used but could be
  public static boolean isEmergencyStopped() {

    if (RobotNav.getGyro().getVelocityX() > 1.0 || RobotNav.getGyro().getVelocityY() > 1.0
        || RobotNav.getGyro().getVelocityZ() > 1.0 || RobotNav.getLeftEncoderVelocity() > 1.0 || RobotNav.getRightEncoderVelocity() > 1.0) {
      Drivetrain drivetrain = new Drivetrain();
      drivetrain.emergencyStop();
    }
    return false;
  }

}
