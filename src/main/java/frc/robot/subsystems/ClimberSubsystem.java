package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import frc.robot.lib.RoboLionsPID;
import frc.robot.Constants.ClimbConstants;

public class ClimberSubsystem extends SubsystemBase {

    public static final double LET_OUT_POWER = 0.6; // placeholder
    public static final double PULL_IN_POWER = -0.6; // placeholder

    private static WPI_TalonSRX climbMotor = RobotMap.climberMotor;
    public RoboLionsPID climbPID = new RoboLionsPID();

    public double climb_enc_readout = 0;

    public ClimberSubsystem() {
        climbMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        resetEncoder();
        climbPID.initialize(ClimbConstants.kP, // Proportional Gain
                            ClimbConstants.kI, // Integral Gain
                            ClimbConstants.kD, // Derivative Gain
                            20, // Cage Limit
                            2, // Deadband
                            0.75 // MaxOutput
        );
    }

    public void moveClimbToPosition(double target_position) {
        climb_enc_readout = getEncoderPosition();
        double arm_cmd = climbPID.execute((double)target_position, (double)climb_enc_readout);
        climbMotor.set(-arm_cmd); // need to invert command to close the loop
    }

    public void setClimbToColorWheel() {
        moveClimbToPosition(ClimbConstants.COLOR_WHEEL_POSITION);
    }

    public void setClimbToMax() {
        moveClimbToPosition(ClimbConstants.MAX_POSITION);
    }

    public void setClimbToReady() {
        moveClimbToPosition(ClimbConstants.READY_POSITION);
    }

    public void setClimbToGround() {
        moveClimbToPosition(ClimbConstants.DOWN_POSITION);
    }

    public void setClimbPower(double power) {
        climbMotor.set(power);
    }

    public void stop() {
        climbMotor.set(0);
    }

    public void resetEncoder() {
        climbMotor.setSelectedSensorPosition(0);
    }

    public double getEncoderPosition() {
        return climbMotor.getSelectedSensorPosition();
    }
}