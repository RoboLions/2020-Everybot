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
public class AutoPath1Mirror extends SequentialCommandGroup {
  /**
   * Creates a new Trench baseline
   */
  public AutoPath1Mirror(final DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, ArmSubsystem armSubsystem) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(new AutoMove(driveSubsystem, Mode.DISTANCE, 10, 0.6), new AutoMoveArm(armSubsystem, Position.SCORE),
    //move to target zone and lower arm
        new Outtake(intakeSubsystem), new AutoTurn(driveSubsystem, 180, 0.6),
        //dump and then turn around
        new AutoMove(driveSubsystem, Mode.DISTANCE, 10, 0.6), new AutoTurn(driveSubsystem, 45, 0.6),
        //head to autoline and turn 
        new AutoMove(driveSubsystem, Mode.DISTANCE, 10, 0.6), new AutoTurn(driveSubsystem, -45, 0.6),
        //turn left and head to trench
        new AutoMove(driveSubsystem, Mode.DISTANCE, 15, 0.6), new Intake(intakeSubsystem, Mode.TIME, 1)); 
        //go further into the trench and suck up powercells
  }
}