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
public class AutoPath5 extends SequentialCommandGroup {
  /**
   * Creates a middle 2 cycles.
   */
  public AutoPath5(final DriveSubsystem driveSubsystem, final IntakeSubsystem intakeSubsystem, ArmSubsystem armSubsystem) {
 //move straight
 super(new AutoMove(driveSubsystem, 2.3), new StopNWait(driveSubsystem, 0.3),
    //turn right
    new AutoTurn(driveSubsystem, 90, 0.6), new StopNWait(driveSubsystem, 0.3), 
    //move straight
    new AutoMove(driveSubsystem, 2.585), new StopNWait(driveSubsystem, 0.3),
    //move left
    new AutoTurn(driveSubsystem, -90), new StopNWait(driveSubsystem, 0.3),
    //move straight
    new AutoMove(driveSubsystem, 0.55), new StopNWait(driveSubsystem, 0.3),
    //dump powercells
    new Outtake(intakeSubsystem).withTimeout(1.5), new StopNWait(driveSubsystem, 0.3),
    //turn left
    new AutoTurn(driveSubsystem, -170), new StopNWait(driveSubsystem, 0.3),
    //move straight pass the autoline
    new AutoMove(driveSubsystem, 5));
}
}

