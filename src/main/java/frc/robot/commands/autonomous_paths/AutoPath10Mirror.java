/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomous_paths;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoMove;
import frc.robot.commands.AutoMoveArm;
import frc.robot.commands.AutoTurn;
import frc.robot.commands.Intake;
import frc.robot.commands.Outtake;
import frc.robot.commands.StopNWait;
import frc.robot.commands.AutoMove.Mode;
import frc.robot.commands.AutoMoveArm.Position;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoPath10Mirror extends SequentialCommandGroup {
  /**
   * Creates trench 2 cycles.
   */
  public AutoPath10Mirror(final DriveSubsystem driveSubsystem, final IntakeSubsystem intakeSubsystem, ArmSubsystem armSubsystem) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(
        //move straight 
        new AutoMove(driveSubsystem, 2.8), new StopNWait(driveSubsystem, 0.1),
        //dump
        new Outtake(intakeSubsystem).withTimeout(1.5), new StopNWait(driveSubsystem, 0.1),
        //move back
        new AutoMove(driveSubsystem, -0.5), new StopNWait(driveSubsystem, 0.1),
        //turn right
        new AutoTurn(driveSubsystem, 160), new StopNWait(driveSubsystem, 0.1),
        //move straight
        new AutoMove(driveSubsystem, 2), new StopNWait(driveSubsystem, 0.3),
        //turn right
        new AutoTurn(driveSubsystem, 25), new StopNWait(driveSubsystem, 0.1),
        //move straight
        new AutoMove(driveSubsystem, 2), new StopNWait(driveSubsystem, 0.3),
        //turn right
        new AutoTurn(driveSubsystem, 90), new StopNWait(driveSubsystem, 0.1),
        //move straight
        new AutoMove(driveSubsystem, 0.4), new StopNWait(driveSubsystem, 0.3),
        //turn right
        new AutoTurn(driveSubsystem, 90), new StopNWait(driveSubsystem, 0.1),
        //move straight
        new AutoMove(driveSubsystem, 2), new StopNWait(driveSubsystem, 0.3));
  }
}


