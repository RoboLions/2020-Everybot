/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.AutonomousPaths;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoMove;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutoMoveArm;
import frc.robot.commands.AutoTurn;
import frc.robot.commands.Intake;
import frc.robot.commands.StopNWait;
import frc.robot.commands.AutoMoveArm.Position;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class TestPath extends SequentialCommandGroup {
  /**
   * Robot moves from the initiation line towards the goal and then emptys into the goal
   */

  public TestPath(final DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, ArmSubsystem armSubsystem) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());  
    super(new AutoMove(driveSubsystem, 3),
      new AutoTurn(driveSubsystem, 180),
      new AutoMove(driveSubsystem, 3));
      /*,new StopNWait(driveSubsystem, 0.3),
      new AutoTurn(driveSubsystem, -20),
      new StopNWait(driveSubsystem, 0.3),
      new AutoMove(driveSubsystem, 1));
      */
        //new Intake(intakeSubsystem).withTimeout(1),
        //new AutoMoveArm(armSubsystem, Position.GROUND),
        //new AutoMove(driveSubsystem, 1));
  }
}
