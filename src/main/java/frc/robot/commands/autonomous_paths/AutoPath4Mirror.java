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
import frc.robot.commands.AutoMove.Mode;
import frc.robot.commands.AutoMoveArm.Position;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoPath4Mirror extends SequentialCommandGroup {
  /**
   * Creates a new Far 2 cycles.
   */
  public AutoPath4Mirror(final DriveSubsystem driveSubsystem, final IntakeSubsystem intakeSubsystem, ArmSubsystem armSubsystem) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(new AutoMove(driveSubsystem, Mode.DISTANCE, 8.75, 0.6), new AutoTurn(driveSubsystem, -90, 0.6),
    //move towards target zone
        new AutoMove(driveSubsystem, Mode.DISTANCE, 12.5, 0.6), new  AutoTurn(driveSubsystem, 90, 0.6),
        //position infront of target zone
        new AutoMove(driveSubsystem, Mode.DISTANCE, 1.25, 0.6), new AutoMoveArm(armSubsystem, Position.SCORE), 
        //move and then
        new Outtake(intakeSubsystem, Mode.TIME,1), new AutoMove(driveSubsystem, Mode.DISTANCE, -1.25, 0.6),
        //dump into bottom port and go back
        new AutoTurn(driveSubsystem, 90, 0.6), new AutoMove(driveSubsystem, Mode.DISTANCE, 13.75, 0.6),
        //position towards to autoline
        new AutoTurn(driveSubsystem, 90, 0.6), new AutoMove(driveSubsystem, Mode.DISTANCE, 8.75, 0.6),
        //land on the autoline
        new AutoTurn(driveSubsystem, 45, 0.6), new AutoMove(driveSubsystem, Mode.DISTANCE, 10, 0.6),
        //land on autoline 
        new AutoMoveArm(armSubsystem, Position.GROUND), new Intake(intakeSubsystem, Mode.TIME,1),
        // lower arm and collect powercells
        new AutoTurn(driveSubsystem, 25, 0.6), new AutoMove(driveSubsystem, Mode.DISTANCE, 5, 0.6), 
        //move to the target zone
        new AutoMoveArm(armSubsystem, Position.SCORE), new Outtake(intakeSubsystem, Mode.TIME,1));
        //lower arm and then dump
}
}

