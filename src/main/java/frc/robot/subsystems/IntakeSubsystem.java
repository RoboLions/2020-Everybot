package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class IntakeSubsystem extends SubsystemBase {

    public static final double INTAKE_POWER = 0.2; // placeholder
    public static final double STOP_POWER = 0.0;

    private static final WPI_TalonSRX intakeMotor = RobotMap.intakeMotor;

    public IntakeSubsystem() {
        intakeMotor.setNeutralMode(NeutralMode.Brake);
    }

    public void intakeBalls() {
        intakeMotor.set(INTAKE_POWER);
    }

    public void outtakeBalls() {
        intakeMotor.set(-INTAKE_POWER);
    }

    public void stop() {
        intakeMotor.set(STOP_POWER);
    }
}