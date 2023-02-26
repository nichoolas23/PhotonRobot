// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import static frc.robot.Constants.FieldConstants.FIRST_BLUE_GRID;
import static frc.robot.PhysicalInputs.XBOX_CONTROLLER;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utilities.TrajectoryGen;


public final class Auto
{

  /** Example static factory for an autonomous command. */
  public static CommandBase autoFactory(Drivetrain drivetrain)
  {
    return Commands.sequence( TrajectoryGen.getTrajCmd(drivetrain),new AlignWithBlockGridCmd(drivetrain,XBOX_CONTROLLER,6)).alongWith(new PathFindCommand(drivetrain,FIRST_BLUE_GRID));//new PathFollowCmd()
  }



  private Auto()
  {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
