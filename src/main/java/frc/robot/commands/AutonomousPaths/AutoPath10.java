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
import frc.robot.commands.AutoMove.Mode;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoPath10 extends SequentialCommandGroup {
  /**
   * Creates trench 2 cycles.
   */
  public AutoPath10(final DriveSubsystem driveSubsystem, final IntakeSubsystem intakeSubsystem) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(new AutoTurn(driveSubsystem, 12, 0.6), new AutoMove(driveSubsystem, Mode.DISTANCE, 10, 0.6),
    //position towards the target zone
        new Intake(intakeSubsystem, Mode.TIME,1), new AutoTurn(driveSubsystem, 180, 0.6),
        //dump and turn around
        new AutoMove(driveSubsystem, Mode.DISTANCE, 25, 0.6), new AutoTurn(driveSubsystem, -27, 0.6), 
        //go to the trench run
        new Intake(intakeSubsystem, Mode.TIME,1), new AutoMove(driveSubsystem, Mode.DISTANCE, 10, 0.6),
        //pick up powercells
        new AutoTurn(driveSubsystem, -90, 0.6), new AutoMove(driveSubsystem, Mode.DISTANCE, 5, 0.6),
        //began to make a u turn
        new AutoTurn(driveSubsystem, -90, 0.6), new AutoMove(driveSubsystem, Mode.DISTANCE, 12.5, 0.6),
        //heading towards autoline
        new AutoTurn(driveSubsystem, 13, 0.6), new Intake(intakeSubsystem, Mode.TIME,1));
        //head to bottom port and dump
  }
}
