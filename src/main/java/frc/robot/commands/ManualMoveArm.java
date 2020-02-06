package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import frc.robot.subsystems.ArmSubsystem;

public class ManualMoveArm extends CommandBase {
    public static double ARM_POWER = 0.6;
    private final ArmSubsystem armSubsystem;
    private final XboxController driverController = RobotContainer.driverController;

    public ManualMoveArm(ArmSubsystem arm) {
        armSubsystem = arm;
    }

    @Override
    public void initialize() {
        armSubsystem.stop();
    }

    @Override
    public void execute() {
        double throttle = driverController.getY(Hand.kLeft);

        if (Math.abs(throttle) > 0.1) {
            RobotMap.armMotor.set(throttle);
        } else {
            RobotMap.armMotor.set(0.0);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}