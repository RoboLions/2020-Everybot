package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;

public class JoystickDrive extends CommandBase {
    private final DriveSubsystem driveSubsystem;
    private final XboxController driverController = RobotContainer.driverController;

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
        double throttle = driverController.getY(Hand.kLeft);
        double rotate = driverController.getX(Hand.kRight);

        driveSubsystem.drive(throttle, rotate);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}