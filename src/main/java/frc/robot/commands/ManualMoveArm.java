package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import frc.robot.subsystems.ArmSubsystem;

public class ManualMoveArm extends CommandBase {
    public static double ARM_POWER = 0.6;
    private final ArmSubsystem armSubsystem;
    private final XboxController driverController = RobotContainer.manipulatorController;

    public static int wrist_motion_state = 0;

    public ManualMoveArm(ArmSubsystem arm) {
        armSubsystem = arm;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double armPower = driverController.getY(Hand.kLeft);
        boolean left_trigger = driverController.getTriggerAxis(Hand.kLeft) > 0.1;
        // left trigger = ground
        boolean right_trigger = driverController.getTriggerAxis(Hand.kRight) > 0.1;
        // right trigger = score

        switch(wrist_motion_state) {
            case 0:
                if(armPower < 0.25 && armPower > -0.25) {
                    armPower = 0;
                }
                armSubsystem.setArmPower(armPower);
                if(armSubsystem.armPID.deadband_active) {
                    wrist_motion_state = 0;
                }
                if(left_trigger) {
                    wrist_motion_state = 1;
                }
                if(right_trigger) {
                    wrist_motion_state = 2;
                }
                break;
            case 1:
            // ground
                armSubsystem.setArmToGround();
                if(armSubsystem.armPID.deadband_active) {
                    wrist_motion_state = 0;
                }                
                break;
            case 2:
            // score
                armSubsystem.setArmToScore();
                if(armSubsystem.armPID.deadband_active) {
                    wrist_motion_state = 0;
                }                
                break;
            default:
                wrist_motion_state = 0;
                break;
        }
        if(Math.abs(armPower) > 0.5 ) {
            wrist_motion_state = 0;
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}