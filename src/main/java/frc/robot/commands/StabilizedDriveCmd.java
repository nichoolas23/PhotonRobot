package frc.robot.commands;

import static frc.robot.Constants.RobotConstants.STAB_PID_D;
import static frc.robot.Constants.RobotConstants.STAB_PID_I;
import static frc.robot.Constants.RobotConstants.STAB_PID_P;
import static frc.robot.PhysicalInputs.XBOX_CONTROLLER;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utilities.RobotNav;

public class StabilizedDriveCmd extends PIDCommand {

  /**
   * Creates a new StabilizedDriveCmd.
   */
  private Drivetrain _drivetrain;
  private XboxController _controller;
  public StabilizedDriveCmd(Drivetrain drivetrain, XboxController controller,RobotNav robotNav) {
    super(
          new PIDController(
              STAB_PID_P,
              STAB_PID_I,
              STAB_PID_D),
          // Close the loop on the turn rate
        robotNav::getTurnRate,
        // Setpoint is 0
        0,
        // Pipe the output to the turning controls
        output -> drivetrain.drive(controller.getRightTriggerAxis(),controller.getLeftTriggerAxis(), output),

        // Require the robot drive
        drivetrain);

    SmartDashboard.putData("Stab pid",this.getController());
// CTR: CAN frame not received/too-stale
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements();
  }

  // Called when the command is initially scheduled.

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   XBOX_CONTROLLER.setRumble(RumbleType.kBothRumble,.50);

   XBOX_CONTROLLER.setRumble(RumbleType.kBothRumble,0);
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {


    //DriverStation.reportError((_controller.getRightX() != 0) +"",true);
    return false;
  }


}
