package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import frc.robot.RobotMap;
import frc.robot.commands.JoystickDrive;
import frc.robot.lib.RoboLionsPID;

public class DriveSubsystem extends SubsystemBase {
    public static final double kMaxSpeed = 3.0; // 3.0; // meters per second
    public static final double kMaxAngularSpeed = 2 * Math.PI; // 2 * Math.PI; // one rotation per second
    private static final double IN_TO_M = .0254;
    
    /* Dev Bot Variables */
    private static final int timeoutMs = 10;
    private static final int MOTOR_ENCODER_CODES_PER_REV = 4096; //4096 for CTRE Mag Encoders
    private static final double DIAMETER_INCHES = 6.0; // dev bot, inches
    private static final double kTrackWidth = 0.35 * 2; 
    private static final double kWheelRadius = 2.5 * IN_TO_M; // in meters, equal to 0.25 inches

    /* Poomba dev variables
    public static final int MOTOR_ENCODER_CODES_PER_REV = 4096;  //4096 for CTRE Mag Encoders
    public static final double DIAMETER_INCHES = 6.0;//2019 6 // 2018 10 // bot 2 = 7.5 poomba, inches
    public static final double WHEEL_DIAMETER_2019 = DIAMETER_INCHES * IN_TO_M; // in meters
    */

    /* Everybot variables 
    private static final int MOTOR_ENCODER_CODES_PER_REV = 2048; // Falcon 500 has 2048 CPR Encoders
    private static final double DIAMETER_INCHES = 5.0; // Everybot, inches
    */
    
	private static final double WHEEL_DIAMETER = DIAMETER_INCHES * IN_TO_M; // in meters
	private static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
	private static final double TICKS_PER_METER = MOTOR_ENCODER_CODES_PER_REV / WHEEL_CIRCUMFERENCE;
    private static final double METERS_PER_TICKS = 1 / TICKS_PER_METER;
    
    private static final WPI_TalonSRX leftMotorFront = RobotMap.leftMotorFront;
    private static final WPI_TalonSRX leftMotorBack = RobotMap.leftMotorBack;
    private static final WPI_TalonSRX rightMotorFront = RobotMap.rightMotorFront;
    private static final WPI_TalonSRX rightMotorBack = RobotMap.rightMotorBack;

    private final PigeonIMU imu = RobotMap.imu;

    private static final SpeedControllerGroup m_leftGroup = new SpeedControllerGroup(leftMotorFront, leftMotorBack);
    private static final SpeedControllerGroup m_rightGroup = new SpeedControllerGroup(rightMotorFront, rightMotorBack);

    private final DifferentialDrive m_drive = new DifferentialDrive(m_leftGroup, m_rightGroup);

    private final PIDController m_leftPIDController = new PIDController(1, 0, 0); // need to tune
    private final PIDController m_rightPIDController = new PIDController(1, 0, 0); // need to tune

    private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(kTrackWidth);

    // Gains are for example purposes only - must be determined for your own robot!
    // private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1, 3); // need to tune
    public SimpleMotorFeedforward m_leftFeedForward;
    public SimpleMotorFeedforward m_rightFeedForward;

    public RoboLionsPID leftForwardPID = new RoboLionsPID();
    public RoboLionsPID rightForwardPID = new RoboLionsPID();
	public RoboLionsPID headingPID = new RoboLionsPID();
	public RoboLionsPID limelightPID = new RoboLionsPID();

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

            /**
     * Returns the angle of the robot as a Rotation2d.
     *
     * @return The angle of the robot.
     */
    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(getYaw());
    }

    /**
     * Sets the desired wheel speeds.
     *
     * @param speeds The desired wheel speeds.
     */
    public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
        /*
        // computing voltage command to send to Talons based on the setpoint speed ie. how fast we want to go
        final double leftFeedforward = m_feedforward.calculate(speeds.leftMetersPerSecond);
        final double rightFeedforward = m_feedforward.calculate(speeds.rightMetersPerSecond);

        final double leftOutput = m_leftPIDController.calculate(getLeftEncoderVelocity(), speeds.leftMetersPerSecond);
        final double rightOutput = m_rightPIDController.calculate(getRightEncoderVelocity(), speeds.rightMetersPerSecond);
        m_leftGroup.setVoltage(leftOutput + leftFeedforward);
        m_rightGroup.setVoltage(rightOutput + rightFeedforward);
        */

        final double leftFeedforward = m_leftFeedForward.calculate(speeds.leftMetersPerSecond);
        final double rightFeedforward = m_rightFeedForward.calculate(speeds.rightMetersPerSecond);

        double leftOutput = leftForwardPID.execute(speeds.leftMetersPerSecond, getLeftEncoderVelocityMetersPerSecond());
        double rightOutput = rightForwardPID.execute(speeds.rightMetersPerSecond, getRightEncoderVelocityMetersPerSecond());
        // m_leftGroup.setVoltage(leftOutput + leftFeedforward);
        // m_rightGroup.setVoltage(rightOutput + rightFeedforward);
        // m_leftGroup.setVoltage(JoystickDrive.throttle);
        // m_rightGroup.setVoltage(JoystickDrive.throttle);

        System.out.println("Debug Out  " + rightOutput + " /// " + rightFeedforward + " /// " + JoystickDrive.throttle);
    }

    @Override
    public void periodic() {

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