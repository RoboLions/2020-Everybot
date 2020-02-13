package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.lib.RoboLionsPID;

public class ArmSubsystem extends SubsystemBase {

    private final WPI_TalonSRX armMotor = RobotMap.armMotor;
    public RoboLionsPID armPID = new RoboLionsPID();
    private final PigeonIMU imu = RobotMap.arm_imu;

    public double arm_pitch_readout = 0;

    public ArmSubsystem() {
        armPID.initialize(ArmConstants.kP, // Proportional Gain
                            ArmConstants.kI, // Integral Gain
                            ArmConstants.kD, // Derivative Gain
                            20, // Cage Limit
                            2, // Deadband
                            0.75 // MaxOutput
        );
    }

    public void moveArmToPosition(double target_pitch) {
        arm_pitch_readout = getPitch();
        double arm_cmd = armPID.execute((double)target_pitch, (double)arm_pitch_readout);
        armMotor.set(-arm_cmd); // need to invert command to close the loop
    }

    public void setArmToGround() {
        moveArmToPosition(ArmConstants.GROUND_POSITION);
    }

    public void setArmToScore() {
        moveArmToPosition(ArmConstants.SCORE_POSITION);
    }

    public void setArmPower(double power) {
        armMotor.set(power);
    }

    public void stop() {
        armMotor.set(0);
    }

    public double getPitch() {
    	double[] ypr = new double[3];
    	imu.getYawPitchRoll(ypr);
    	return ypr[1];
    }

    public void resetPitch() {
        imu.setYaw(0, ArmConstants.TIMEOUT_MS);
    }
}