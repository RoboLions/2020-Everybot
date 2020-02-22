/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.AutonomousPaths;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoMove;
import frc.robot.commands.AutoTurn;
import frc.robot.commands.Intake;
import frc.robot.commands.Outtake;
import frc.robot.commands.AutoMove.Mode;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoPath1Mirror extends SequentialCommandGroup {
  /**
   * Creates a new Trench baseline
   */
  public AutoPath1Mirror(final DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(new AutoMove(driveSubsystem, Mode.DISTANCE, 10, 0.6), new Outtake(intakeSubsystem),
    //move to target zone and dump
        new AutoTurn(driveSubsystem, 180, 0.6), new AutoMove(driveSubsystem, Mode.DISTANCE, 10, 0.6),
        //turn around and return
        new AutoTurn(driveSubsystem, 45, 0.6), new AutoMove(driveSubsystem, Mode.DISTANCE, 10, 0.6),
        //turn left and head to trench
        new AutoTurn(driveSubsystem, -45, 0.6), new AutoMove(driveSubsystem, Mode.DISTANCE, 15, 0.6),
        //turn right and go further into the trench
        new Intake(intakeSubsystem, Mode.TIME,1)); 
        //suck up powercells
  }
}