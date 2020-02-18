package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;/**
 *
 */
public class IntakeBalls extends CommandBase {
    private final IntakeSubsystem intakeSubsystem;
    private double direction;
    // direction: -1 is inwards, 1 is outwards
    
    public IntakeBalls(IntakeSubsystem intake, double dir) {
        direction = dir;
        intakeSubsystem = intake;
        addRequirements(intakeSubsystem);
      }

    @Override
    public void initialize() {
    	
    }

    @Override
    public void execute() {
        if(direction == -1) {
            intakeSubsystem.intakeBalls();
        } else if(direction == 1) {
            intakeSubsystem.outtakeBalls();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}