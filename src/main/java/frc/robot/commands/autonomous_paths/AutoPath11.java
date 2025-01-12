/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomous_paths;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Outtake;
import frc.robot.commands.StopNWait;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoPath11 extends SequentialCommandGroup {
  /**
   * Creates a new feedingpath.
   */
  public AutoPath11(final DriveSubsystem driveSubsystem, final IntakeSubsystem intakeSubsystem) {
    // Add your commands in the super() call, e.g.
    // wait then dump
    super(new StopNWait(driveSubsystem, 3.0), new Outtake(intakeSubsystem).withTimeout(1.5));
  }
}
