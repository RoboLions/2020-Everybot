package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import frc.robot.RobotMap;
import frc.robot.commands.JoystickDrive;

public class DriveSubsystem extends SubsystemBase {
    public static final double kMaxSpeed = 3.0; // 3.0; // meters per second
    public static final double kMaxAngularSpeed = 2 * Math.PI; // 2 * Math.PI; // one rotation per second

    private static final double kTrackWidth = 0.381 * 2; // TODO find this, and what it represents
    private static final double kWheelRadius = 0.0635; // in meters, equal to 0.25 inches
    // private static final int kEncoderResolution = 2048; // Falcon 500 has 2048 CPR Encoders
    
    private static final int timeoutMs = 10;
    private static final int kEncoderResolution = 4096; //4096 for CTRE Mag Encoders
    private static final double DIAMETER_INCHES = 5.0; // inches
	private static final double IN_TO_M = .0254;
	private static final double WHEEL_DIAMETER_2020 = DIAMETER_INCHES * IN_TO_M; // in meters
	private static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER_2020 * Math.PI;
	private static final double TICKS_PER_METER = kEncoderResolution / WHEEL_CIRCUMFERENCE;
	private static final double METERS_PER_TICKS = 1 / TICKS_PER_METER;

    private static final WPI_TalonSRX leftMotorFront = RobotMap.leftMotorFront;
    private static final WPI_TalonSRX leftMotorBack = RobotMap.leftMotorBack;
    private static final WPI_TalonSRX rightMotorFront = RobotMap.rightMotorFront;
    private static final WPI_TalonSRX rightMotorBack = RobotMap.rightMotorBack;

    private final PigeonIMU imu = RobotMap.imu;

    private static final SpeedControllerGroup m_leftGroup = new SpeedControllerGroup(leftMotorFront, leftMotorBack);
    private static final SpeedControllerGroup m_rightGroup = new SpeedControllerGroup(rightMotorFront, rightMotorBack);

    private final DifferentialDrive m_drive = new DifferentialDrive(m_leftGroup, m_rightGroup);

    public DriveSubsystem() {
        ZeroYaw();
        resetEncoders();

        leftMotorFront.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 1);
        leftMotorFront.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        
        rightMotorFront.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 1);
        rightMotorFront.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        
        leftMotorBack.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 1);
        leftMotorBack.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        
        rightMotorBack.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 1);
		rightMotorBack.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        
        m_leftGroup.setInverted(true);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Yaw", getYaw());
        SmartDashboard.putNumber("Pitch", getPitch());
        SmartDashboard.putNumber("Roll", getRoll());
        SmartDashboard.putNumber("Left Position", getLeftEncoderPosition());
        SmartDashboard.putNumber("Right Position", getRightEncoderPosition());
        SmartDashboard.putNumber("Left Velocity", getLeftEncoderVelocity());
        SmartDashboard.putNumber("Right Velocity", getRightEncoderVelocity());

    }

    public void setModeVoltage() {
        leftMotorFront.set(ControlMode.PercentOutput, 0);
        rightMotorFront.set(ControlMode.PercentOutput, 0);
    }
    
    public double getYaw() {
    	double[] ypr = new double[3];
    	imu.getYawPitchRoll(ypr);
    	return -ypr[0];
    }
    
    public double getPitch() {
    	double[] ypr = new double[3];
    	imu.getYawPitchRoll(ypr);
    	return ypr[1];
    }
    
    public double getRoll() {
    	double[] ypr = new double[3];
    	imu.getYawPitchRoll(ypr);
    	return ypr[2];
    }

    
    public double[] getRPH() {
    	double[] ypr = new double[3];
		imu.getYawPitchRoll(ypr);
		ypr[0] = -ypr[0];
    	return(ypr);
    }

    public void ZeroYaw() {
    	imu.setYaw(0, timeoutMs);
    	imu.setFusedHeading(0, timeoutMs);
    }

    public double distanceTravelled() {
		return (getLeftEncoderPosition() + getRightEncoderPosition()) / 2;
	}

	public double getLeftEncoderPosition() {
		return leftMotorFront.getSelectedSensorPosition();
	}

	public double getRightEncoderPosition() {
		return rightMotorFront.getSelectedSensorPosition();
	}
	
	public double getLeftEncoderVelocity() {
		return leftMotorFront.getSelectedSensorVelocity();
	}

	public double getRightEncoderVelocity() {
		return rightMotorFront.getSelectedSensorVelocity();
    }	
    
    public double getLeftEncoderVelocityMetersPerSecond() {
        //getQuadVelocity is in 100 ms so we have to divide it by 10 to get seconds
        double leftVelocityMPS = (leftMotorFront.getSelectedSensorVelocity()/10); 
        // since getQuadVelocity is in encoders, we have to convert it to meters
        leftVelocityMPS = leftVelocityMPS * METERS_PER_TICKS;
        return (leftVelocityMPS);
    }
    
    public double getRightEncoderVelocityMetersPerSecond() {
        //getQuadVelocity is in 100 ms so we have to divide it by 10 to get seconds
        double rightVelocityMPS = (rightMotorFront.getSelectedSensorVelocity()/10); 
        // since getQuadVelocity is in encoders, we have to convert it to meters
        rightVelocityMPS = rightVelocityMPS * METERS_PER_TICKS;
        return (rightVelocityMPS);
    }

	public double distanceTravelledinMeters() {
		double left_dist = getLeftEncoderPosition() * METERS_PER_TICKS;
		double right_dist = getRightEncoderPosition() * METERS_PER_TICKS;
		return (left_dist + right_dist) / 2;
	}

	public void resetEncoders() {
		leftMotorFront.setSelectedSensorPosition(0);
        leftMotorBack.setSelectedSensorPosition(0);
		rightMotorFront.setSelectedSensorPosition(0);
		rightMotorBack.setSelectedSensorPosition(0);
    }
    
    public static void drive(double throttle, double rotate) {
        m_leftGroup.set(throttle + rotate);
        m_rightGroup.set(throttle - rotate);
    }

    public void stop() {
        drive(0, 0);
    }
}