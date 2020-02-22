/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomous_paths;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoMove;
import frc.robot.commands.AutoMove.Mode;
import frc.robot.commands.AutoTurn;
import frc.robot.commands.Intake;
import frc.robot.commands.Outtake;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoPath6 extends SequentialCommandGroup {
  /**
   * Creates a new Trench (5 Balls).
   */
  public AutoPath6(final DriveSubsystem driveSubsystem, final IntakeSubsystem intakeSubsystem) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(new AutoTurn(driveSubsystem, 10, 0.6), new AutoMove(driveSubsystem, Mode.DISTANCE, 10, 0.6),
    //position to bottom port
        new Outtake(intakeSubsystem, Mode.TIME,1), new AutoTurn(driveSubsystem, 180, 0.6),
        //dump and turn around
        new AutoMove(driveSubsystem, Mode.DISTANCE, 20, 0.6), new AutoTurn(driveSubsystem, 45, 0.6),
        //go to trench run
        new AutoMove(driveSubsystem, Mode.DISTANCE, 10, 0.6), new Intake(intakeSubsystem, Mode.TIME,1));
        // move through the trench run while sucking in balls and
  }
}
