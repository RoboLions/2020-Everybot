package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class IntakeSubsystem extends SubsystemBase {

    public static final double IN_POWER = 0.6; // placeholder
    public static final double OUT_POWER = -0.6; // placeholder

    private static final WPI_TalonSRX intakeMotor = RobotMap.intakeMotor;

    public void intakeBalls() {
        intakeMotor.set(IN_POWER);
    }

    public void outtakeBalls() {
        intakeMotor.set(OUT_POWER);
    }

    public void stop() {
        intakeMotor.set(0);
    }
}