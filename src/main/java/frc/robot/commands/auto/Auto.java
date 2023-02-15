// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Drivetrain;


public final class Auto
{
  /** Example static factory for an autonomous command. */
  public static CommandBase autoFactory(Drivetrain drivetrain)
  {
    return Commands.sequence(new PathFindCommand(drivetrain));//new PathFollowCmd()
  }


  private Auto()
  {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
