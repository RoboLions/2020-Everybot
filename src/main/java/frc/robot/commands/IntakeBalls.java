package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj.XboxController;

/**
 *
 */
public class IntakeBalls extends CommandBase {
    private final IntakeSubsystem intakeSubsystem;
    private final static XboxController manipulatorController = RobotContainer.manipulatorController;
    
    public IntakeBalls(IntakeSubsystem intake) {
        intakeSubsystem = intake;
        addRequirements(intakeSubsystem);
      }

    @Override
    public void initialize() {
    	
    }

    @Override
    public void execute() {
        boolean start = manipulatorController.getStartButton(); // outtake
        boolean back = manipulatorController.getBackButton(); // intake

        if(back) {
            intakeSubsystem.intakeBalls();
        } else if(start) {
            intakeSubsystem.outtakeBalls();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stop();
    }
}