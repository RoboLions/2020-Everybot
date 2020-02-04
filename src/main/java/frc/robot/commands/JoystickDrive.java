package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;

public class JoystickDrive extends CommandBase {
    private final DriveSubsystem driveSubsystem;
    private final static XboxController driverController = RobotContainer.driverController;
    public static double throttle = driverController.getY(Hand.kLeft);
    public double rotate = driverController.getX(Hand.kRight);

    public JoystickDrive(DriveSubsystem drivetrain) {
        driveSubsystem = drivetrain;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        driveSubsystem.stop();
    }

    @Override
    public void execute() {

        // PID Testing Joystick Inputs
        if (driverController.getY(Hand.kLeft) > 0.2) 
        {
            throttle = 0.5;
        } 
        else if (driverController.getY(Hand.kLeft) < -0.2) {
            throttle = -0.5;
        }
        else {
            throttle = 0.0;
        }
        rotate = 0.0;

        driveSubsystem.drive(throttle, rotate);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}