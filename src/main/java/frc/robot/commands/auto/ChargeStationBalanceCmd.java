package frc.robot.commands.auto;

import static frc.robot.Constants.RobotConstants.STAB_PID_D;
import static frc.robot.Constants.RobotConstants.STAB_PID_I;
import static frc.robot.Constants.RobotConstants.STAB_PID_P;
import static frc.robot.PhysicalInputs.XBOX_CONTROLLER;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utilities.RobotNav;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

public class ChargeStationBalanceCmd extends PIDCommand {


  public ChargeStationBalanceCmd(RobotNav robotNav, Drivetrain drivetrain) {
    super(
        new PIDController(
            STAB_PID_P,
            STAB_PID_I,
            STAB_PID_D),
        // Close the loop on the turn rate
        robotNav::getRobotPitch,
        // Setpoint is 0
        0,
        // Pipe the output to the turning controls
        output -> {
          if (output > 0) {
            drivetrain.drive(output, 0, 0);
          } else if (output < 0) {
            drivetrain.drive(0, output * -1, 0);
          }
        },

        // Require the robot drive
        drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements();
  }

  @Override
  public void initialize() {
   SmartDashboard.putData("Charge Station PID",this.getController());
  }
}

