package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import frc.robot.subsystems.ClimberSubsystem;

public class ManualMoveClimb extends CommandBase {
    public static double POWER = 0.6;
    private final ClimberSubsystem climberSubsystem;
    private final XboxController driverController = RobotContainer.driverController;

    public static int wrist_motion_state = 0;

    public ManualMoveClimb(ClimberSubsystem climb) {
        climberSubsystem = climb;
        addRequirements(climberSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double climbPower;
        
        boolean left_bumper = driverController.getBumper(Hand.kLeft);
        boolean right_bumper = driverController.getBumper(Hand.kRight);

        if(left_bumper) {
            climbPower = POWER;
        } else if(right_bumper) {
            climbPower = -POWER;
        } else {
            climbPower = 0;
        }

        boolean x = driverController.getXButton();
        // x = Color Wheel
        boolean b = driverController.getBButton();
        // b = Max (Tallest possible height at 45 in)
        boolean a = driverController.getAButton();
        // a = Ready (Move to exact height needed to latch on)
        boolean y = driverController.getYButton();
        // y = Ground/Zero Position

        switch(wrist_motion_state) {
            case 0:
                if(climbPower < 0.25 && climbPower > -0.25) {
                    climbPower = 0;
                }
                climberSubsystem.setClimbPower(climbPower);
                if(climberSubsystem.climbPID.deadband_active) {
                    wrist_motion_state = 0;
                }
                if(x) {
                    wrist_motion_state = 1;
                }
                if(b) {
                    wrist_motion_state = 2;
                }
                if(a) {
                    wrist_motion_state = 3;
                }
                if(y) {
                    wrist_motion_state = 4;
                }
                break;
            case 1:
            // Color Wheel
                climberSubsystem.setClimbToColorWheel();
                if(climberSubsystem.climbPID.deadband_active) {
                    wrist_motion_state = 0;
                }                
                break;
            case 2:
            // Max Position
                climberSubsystem.setClimbToMax();
                if(climberSubsystem.climbPID.deadband_active) {
                    wrist_motion_state = 0;
                }                
                break;
            case 3:
            // Ready to latch on
                climberSubsystem.setClimbToReady();
                if(climberSubsystem.climbPID.deadband_active) {
                    wrist_motion_state = 0;
                }
                break;
            case 4:
            // Down to Robot Ground/Base
                climberSubsystem.setClimbToGround();
                if(climberSubsystem.climbPID.deadband_active) {
                    wrist_motion_state = 0;
                }
                break;
            default:
                wrist_motion_state = 0;
                break;
        }
        if(Math.abs(climbPower) > (POWER - 0.1) ) {
            wrist_motion_state = 0;
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}