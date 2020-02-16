package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import frc.robot.subsystems.ClimberSubsystem;

public class ManualMoveClimb extends CommandBase {
    private final ClimberSubsystem climberSubsystem;
    private final XboxController driverController = RobotContainer.driverController;
    private double joystick_climb_motion; // describes how to climb should move based on bumpers
    // 1 = inwards, 2 = outwards, 3 = Not Moving

    public static int climb_motion_state = 0;

    public ManualMoveClimb(ClimberSubsystem climb) {
        climberSubsystem = climb;
        addRequirements(climberSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {        
        boolean left_bumper = driverController.getBumper(Hand.kLeft);
        boolean right_bumper = driverController.getBumper(Hand.kRight);

        if(left_bumper) {
            joystick_climb_motion = 1; // moving inwards
        } else if(right_bumper) {
            joystick_climb_motion = 2; // moving outwards
        } else {
            joystick_climb_motion = 3; // not moving based on bumpers
        }

        boolean x = driverController.getXButton();
        // x = Color Wheel
        boolean b = driverController.getBButton();
        // b = Max (Tallest possible height at 45 in)
        boolean a = driverController.getAButton();
        // a = Ready (Move to exact height needed to latch on)
        boolean y = driverController.getYButton();
        // y = Ground/Zero Position

        switch(climb_motion_state) {
            case 0:
                
                if(joystick_climb_motion == 1) {
                    climberSubsystem.moveClimbIn();
                } else if(joystick_climb_motion == 2) {
                    climberSubsystem.moveClimbOut();
                }

                if(climberSubsystem.climbPID.deadband_active) {
                    climb_motion_state = 0;
                }
                if(x) {
                    climb_motion_state = 1;
                }
                if(b) {
                    climb_motion_state = 2;
                }
                if(a) {
                    climb_motion_state = 3;
                }
                if(y) {
                    climb_motion_state = 4;
                }
                break;
            case 1:
            // Color Wheel
                climberSubsystem.setClimbToColorWheel();
                if(climberSubsystem.climbPID.deadband_active) {
                    climb_motion_state = 0;
                }                
                break;
            case 2:
            // Max Position
                climberSubsystem.setClimbToMax();
                if(climberSubsystem.climbPID.deadband_active) {
                    climb_motion_state = 0;
                }                
                break;
            case 3:
            // Ready to latch on
                climberSubsystem.setClimbToReady();
                if(climberSubsystem.climbPID.deadband_active) {
                    climb_motion_state = 0;
                }
                break;
            case 4:
            // Down to Robot Ground/Base
                climberSubsystem.setClimbToGround();
                if(climberSubsystem.climbPID.deadband_active) {
                    climb_motion_state = 0;
                }
                break;
            default:
                climb_motion_state = 0;
                break;
        }
        if(joystick_climb_motion != 3) {
            climb_motion_state = 0;
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}