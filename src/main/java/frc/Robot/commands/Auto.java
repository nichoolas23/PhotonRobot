// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.Robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.Robot.subsystems.Drivetrain;



public final class Auto
{
  /** Example static factory for an autonomous command. */
  public static CommandBase exampleAuto(Drivetrain subsystem)
  {
    return Commands.sequence(subsystem.exampleMethodCommand(), new Drivetrain(subsystem));
  }


  private Auto()
  {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
