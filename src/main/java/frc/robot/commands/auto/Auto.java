// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import static frc.robot.PhysicalInputs.XBOX_CONTROLLER;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utilities.RobotNav;
import frc.robot.utilities.TrajectoryGen;
import java.util.List;


public final class Auto
{

  /** Example static factory for an autonomous command. */
  public static CommandBase autoFactory(Drivetrain drivetrain, RobotNav robotNav, AprilTag tag,boolean isBlue)
  {
    List<Translation2d> waypoints = switch (tag.ID) { // TODO: Use path weaver to get coords then offset them according to limelight localization used to get around charge station and what not
      case 1,2,3 -> List.of(new Translation2d(-3.148, -3.148), new Translation2d(-5.265, -3.224));
      default -> List.of();
    };
    return Commands.sequence(new InitialDriveOutAutoCmd(drivetrain,robotNav,.2), TrajectoryGen.getTrajCmd(drivetrain,tag,waypoints),new AlignWithBlockGridCmd(drivetrain,XBOX_CONTROLLER,6));//new Path
  }



  private Auto()
  {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
