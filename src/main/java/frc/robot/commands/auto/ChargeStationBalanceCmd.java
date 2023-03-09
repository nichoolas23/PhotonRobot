package frc.robot.commands.auto;

import static frc.robot.Constants.RobotConstants.BALANCE_PID_D;
import static frc.robot.Constants.RobotConstants.BALANCE_PID_I;
import static frc.robot.Constants.RobotConstants.BALANCE_PID_P;
import static frc.robot.Constants.RobotConstants.STAB_PID_D;
import static frc.robot.Constants.RobotConstants.STAB_PID_I;
import static frc.robot.Constants.RobotConstants.STAB_PID_P;
import static frc.robot.PhysicalInputs.XBOX_CONTROLLER;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utilities.RobotNav;

public class ChargeStationBalanceCmd extends ProfiledPIDCommand { // TODO: LOW GEAR PLEASE PLEASE PLEASE omni wheels better not screw this up


  public ChargeStationBalanceCmd(RobotNav robotNav, Drivetrain drivetrain) {
    super(
        new ProfiledPIDController(
            BALANCE_PID_P,
            BALANCE_PID_I,
            BALANCE_PID_D,
            new Constraints(.1,.1)),
        // Close th se loop on the turn rate
        robotNav::getRobotPitch,
        // Setpoint is 0
        0,
        // Pipe the output to the turning controls
        (output, setpoint) -> {

          drivetrain.drive(0, output * -.5, 0);
        },
          // Require the robot drive
          drivetrain);
      getController().enableContinuousInput(-180, 180);
      this.getController().setTolerance(0.2,.1);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements();
  }

  @Override
  public void initialize() {
   SmartDashboard.putData("Charge Station PID",this.getController());
  }
  @Override
  public boolean isFinished(){
    return Math.abs(XBOX_CONTROLLER.getRightTriggerAxis())>.2 || this.getController().atGoal();
  }
}

