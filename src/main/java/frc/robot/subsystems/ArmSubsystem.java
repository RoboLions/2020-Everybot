package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

public class ArmSubsystem extends SubsystemBase {

    private final WPI_TalonSRX armMotor = RobotMap.armMotor;
    
    public static final double ARM_UP_POWER = 0.6; // placeholder 
    public static final double ARM_DOWN_POWER = -0.6; // placeholder 

    public ArmSubsystem() {
        armMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, ArmConstants.TIMEOUT_MS);
    }

    public void moveArm(double power) {
        armMotor.set(power);
    }

    public void armUp() {
        armMotor.set(ARM_UP_POWER);
    }

    public void armDown() {
        armMotor.set(ARM_DOWN_POWER);
    }

    public void stop() {
        armMotor.set(0);
    }

    public double getEncoderPosition() {
        return armMotor.getSelectedSensorPosition();
    }

    public double getArmAngle() {
        double angle_offset = 0;
        double position = Math.abs(getEncoderPosition()); // in encoder ticks
        double angle = (position * 360) / ArmConstants.MAX_ENCODER_COUNTS;
        return(angle + angle_offset); 
    }

    public void resetEncoder() {
        armMotor.setSelectedSensorPosition(0);
    }
}