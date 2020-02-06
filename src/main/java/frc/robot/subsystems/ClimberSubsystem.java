package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class ClimberSubsystem extends SubsystemBase {

    public static final double LET_OUT_POWER = 0.6; // placeholder
    public static final double PULL_IN_POWER = -0.6; // placeholder

    private static WPI_TalonSRX climbMotor = RobotMap.climberMotor;
 
    public void climbUp() {
        climbMotor.set(PULL_IN_POWER);
    }

    public void climbDown() {
        climbMotor.set(LET_OUT_POWER);
    }

    public void stop() {
        climbMotor.set(0);
    }
}