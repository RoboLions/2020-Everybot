package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import frc.robot.RobotContainer;
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

    private final PigeonIMU imu = RobotMap.drive_imu;

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
	//public RoboLionsPID headingPID = new RoboLionsPID();
    //public RoboLionsPID limelightPID = new RoboLionsPID();
    public RoboLionsPID positionPID = new RoboLionsPID();

    public DriveSubsystem() {
        ZeroYaw();
        resetEncoders();

        leftMotorFront.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 1);
        leftMotorFront.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        leftMotorFront.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms);
        leftMotorFront.configVelocityMeasurementWindow(16);//1,2,4,8,16,32,64(default)
        leftMotorFront.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 5, 10);
        
        rightMotorFront.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 1);
        rightMotorFront.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        rightMotorFront.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms);
        rightMotorFront.configVelocityMeasurementWindow(16);//1,2,4,8,16,32,64(default)
        rightMotorFront.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 5, 10);
        
        leftMotorBack.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 1);
        leftMotorBack.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        leftMotorBack.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms);
        leftMotorBack.configVelocityMeasurementWindow(16);//1,2,4,8,16,32,64(default)
        // leftMotorBack.follow(leftMotorFront);
        
        rightMotorBack.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 1);
        rightMotorBack.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        rightMotorBack.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms);
        rightMotorBack.configVelocityMeasurementWindow(16);//1,2,4,8,16,32,64(default)
        // rightMotorBack.follow(rightMotorFront);


        
        m_leftGroup.setInverted(true);
        //m_rightGroup.setInverted(true);

        leftForwardPID.initialize(
        3.5, // Proportional Gain 2 / 7 / ZN 4.2 / 4.2
        20, // Integral Gain 0.0018 / 0.3 / ZN 32.31 / 14
        0, // Derivative Gain ZN 0.1365
        0, // Cage Limit 0.3
        0, // Deadband
        100// MaxOutput 0.25
        );

        rightForwardPID.initialize(
        3.5, // Proportional Gain 0.3 / 7 / ZN 4.2 / 4.2
        20, // Integral Gain 0.0018 / 0.1 / ZN 38 / 14
        0, // Derivative Gain  ZN 0.116
        0, // Cage Limit //0.3
        0, // Deadband
        100// MaxOutput //0.25
        );

        positionPID.initialize(
        0, // Proportional Gain 0.3 / 7 / ZN 4.2 / 4.2 / 1
        0, // Integral Gain 0.0018 / 0.1 / ZN 38 / 14
        0, // Derivative Gain  ZN 0.116
        0, // Cage Limit //0.3
        0, // Deadband
        100// MaxOutput //0.25
        );
    }

    public double calculateNew(double velocity, double acceleration, double ks, double kv, double ka) {
        return ks * Math.signum(velocity) + kv * velocity + ka * acceleration;
    }

    public void straightDrive(double leftSpeed, double rightSpeed) {
        /*
        // computing voltage command to send to Talons based on the setpoint speed ie. how fast we want to go
        final double leftFeedforward = m_feedforward.calculate(speeds.leftMetersPerSecond);
        final double rightFeedforward = m_feedforward.calculate(speeds.rightMetersPerSecond);

        final double leftOutput = m_leftPIDController.calculate(getLeftEncoderVelocity(), speeds.leftMetersPerSecond);
        final double rightOutput = m_rightPIDController.calculate(getRightEncoderVelocity(), speeds.rightMetersPerSecond);
        m_leftGroup.setVoltage(leftOutput + leftFeedforward);
        m_rightGroup.setVoltage(rightOutput + rightFeedforward);
        */
    
        final double leftFeedforward = calculateNew(leftSpeed, 0, 1, 2.6, 0);
        final double rightFeedforward = calculateNew(rightSpeed, 0, 1.2, 2.6, 0);

        double batteryVoltage = RobotController.getBatteryVoltage(); // getting battery voltage from PDP via the rio

        if (batteryVoltage < 1) {
            batteryVoltage = 1;
        }

        //final double leftFeedforward = 0;
        //final double rightFeedforward = 0;

        double leftOutput = leftForwardPID.execute(leftSpeed, getLeftEncoderVelocityMetersPerSecond());
        double rightOutput = rightForwardPID.execute(rightSpeed, getRightEncoderVelocityMetersPerSecond());
        SmartDashboard.putNumber("Left Output", leftOutput);
        SmartDashboard.putNumber("Right Output", rightOutput);

        double LVoltagePercentCommand = ((leftOutput + leftFeedforward) / batteryVoltage);
        double RVoltagePercentCommand = ((rightOutput + rightFeedforward) / batteryVoltage);

        if (LVoltagePercentCommand > 1.0) {
            LVoltagePercentCommand = 1.0;
        }
        else if (LVoltagePercentCommand < -1.0) {
            LVoltagePercentCommand = -1.0;
        }

        if (RVoltagePercentCommand > 1.0) {
            RVoltagePercentCommand = 1.0;
        }
        else if (RVoltagePercentCommand < -1.0) {
            RVoltagePercentCommand = -1.0;
        }

        SmartDashboard.putNumber("Right Motor Command", RVoltagePercentCommand);
        SmartDashboard.putNumber("Left Motor Command", LVoltagePercentCommand);

        leftMotorFront.set(-LVoltagePercentCommand);
        leftMotorBack.set(-LVoltagePercentCommand);
        rightMotorFront.set(RVoltagePercentCommand);
        rightMotorBack.set(RVoltagePercentCommand);

        // m_leftGroup.setVoltage(leftOutput + leftFeedforward);
        // m_rightGroup.setVoltage(rightOutput + rightFeedforward);
        // m_leftGroup.setVoltage(JoystickDrive.throttle);
        // m_rightGroup.setVoltage(JoystickDrive.throttle);

        // System.out.println("Hello World!");
        //System.out.println("LFF " + leftFeedforward + " LPD " + leftOutput + " RFF " + rightFeedforward + " RPD " + rightOutput);
        // There is a + for the error since right encoder velocity was already negative so -- = +
        // System.out.println("Error L - R" + (getLeftEncoderVelocityMetersPerSecond()-getRightEncoderVelocityMetersPerSecond()));
        SmartDashboard.putNumber("Error L - R", (getLeftEncoderVelocityMetersPerSecond()-getRightEncoderVelocityMetersPerSecond()));
        
        SmartDashboard.putNumber("Left Motor Front Voltage", leftMotorFront.getMotorOutputVoltage());
        SmartDashboard.putNumber("Left Motor Back Voltage", leftMotorBack.getMotorOutputVoltage());
        SmartDashboard.putNumber("Right Motor Front Voltage", rightMotorFront.getMotorOutputVoltage());
        SmartDashboard.putNumber("Right Motor Back Voltage", rightMotorBack.getMotorOutputVoltage());
        // System.out.println("L: " + getLeftEncoderVelocityMetersPerSecond() + "/ R:" + getRightEncoderVelocityMetersPerSecond());
        // System.out.println("Left Error: " + (leftSpeed-getLeftEncoderVelocityMetersPerSecond()) + "/ Right Error: " + (getRightEncoderVelocityMetersPerSecond()-rightSpeed));
        // System.out.println("Debug Out  " + rightOutput + " /// " + rightFeedforward + " /// " + JoystickDrive.throttle);
    }

    public void driveWithRotation(double speed, double rotate) {
        final double leftFeedforward = calculateNew(speed, 0, 1, 2.6, 0);
        final double rightFeedforward = calculateNew(speed, 0, 1.2, 2.6, 0);

        double batteryVoltage = RobotController.getBatteryVoltage(); // getting battery voltage from PDP via the rio

        if (batteryVoltage < 1) {
            batteryVoltage = 1;
        }

        //final double leftFeedforward = 0;
        //final double rightFeedforward = 0;

        double leftOutput = leftForwardPID.execute(speed, getLeftEncoderVelocityMetersPerSecond());
        double rightOutput = rightForwardPID.execute(speed, getRightEncoderVelocityMetersPerSecond());
        SmartDashboard.putNumber("Left Output", leftOutput);
        SmartDashboard.putNumber("Right Output", rightOutput);

        double LVoltagePercentCommand = ((leftOutput + leftFeedforward) / batteryVoltage);
        double RVoltagePercentCommand = ((rightOutput + rightFeedforward) / batteryVoltage);

        if (LVoltagePercentCommand > 1.0) {
            LVoltagePercentCommand = 1.0;
        }
        else if (LVoltagePercentCommand < -1.0) {
            LVoltagePercentCommand = -1.0;
        }

        if (RVoltagePercentCommand > 1.0) {
            RVoltagePercentCommand = 1.0;
        }
        else if (RVoltagePercentCommand < -1.0) {
            RVoltagePercentCommand = -1.0;
        }

        SmartDashboard.putNumber("Right Motor Command", RVoltagePercentCommand);
        SmartDashboard.putNumber("Left Motor Command", LVoltagePercentCommand);

        SmartDashboard.putNumber("throttle", speed);
        SmartDashboard.putNumber("rotate", rotate);

        leftMotorFront.set(-LVoltagePercentCommand - rotate);
        leftMotorBack.set(-LVoltagePercentCommand - rotate);
        rightMotorFront.set(RVoltagePercentCommand - rotate);
        rightMotorBack.set(RVoltagePercentCommand - rotate);

        SmartDashboard.putNumber("Error L - R", (getLeftEncoderVelocityMetersPerSecond()-getRightEncoderVelocityMetersPerSecond()));
    }

    /*
    @SuppressWarnings("ParameterName")
        public void drive(double xSpeed, double rot) {
        var wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));
        setSpeeds(wheelSpeeds);
    }
    */

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

    public double distanceTravelledinTicks() {
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
        // inverted because the encoder for dev bot is wired on backwards
    }	
    
    public double getLeftEncoderVelocityMetersPerSecond() {
        //getQuadVelocity is in 100 ms so we have to divide it by 10 to get seconds
        double leftVelocityMPS = (leftMotorFront.getSelectedSensorVelocity()*10); // /10
        // since getQuadVelocity is in encoder ticks, we have to convert it to meters
        leftVelocityMPS = leftVelocityMPS * METERS_PER_TICKS;
        return (leftVelocityMPS);
    }
    
    public double getRightEncoderVelocityMetersPerSecond() {
        //getQuadVelocity is in 100 ms so we have to divide it by 10 to get seconds
        double rightVelocityMPS = (rightMotorFront.getSelectedSensorVelocity()*10); // /10
        // since getQuadVelocity is in encoder ticks, we have to convert it to meters
        //Need to have a negative for right velocity since the motors are reversed on the opposite side
        rightVelocityMPS = -rightVelocityMPS * METERS_PER_TICKS;
        return (rightVelocityMPS);
    }

	public double distanceTravelledinMeters() {
		double left_dist = getLeftEncoderPosition() * METERS_PER_TICKS;
        double right_dist = getRightEncoderPosition() * METERS_PER_TICKS;
        double distanceTravelled = (left_dist + right_dist) / 2;
        SmartDashboard.putNumber("Distance Travelled M", distanceTravelled);
		return distanceTravelled;
	}

	public void resetEncoders() {
		leftMotorFront.setSelectedSensorPosition(0);
        leftMotorBack.setSelectedSensorPosition(0);
		rightMotorFront.setSelectedSensorPosition(0);
		rightMotorBack.setSelectedSensorPosition(0);
    }
    
    // Function for deciding what variables, etc determine the speeds of the drivetrain
    public void driveRoboLionsPID(double throttle, double rotate) {
        driveWithRotation(throttle, rotate);
    }

    // Function for deciding what variables, etc determine the speeds of the drivetrain
    public static void drive(double throttle, double rotate) {
        m_leftGroup.set(throttle + rotate);
        m_rightGroup.set(throttle - rotate);
    }
    
    public void autoDrive(double distance) { // distance is in meters
        double left_speed; 
        double right_speed;

        double position_feedback = distanceTravelledinMeters();

        double output = positionPID.execute(distance, position_feedback);

        left_speed = output;
        right_speed = output;

        straightDrive(left_speed, right_speed);
        System.out.println("TD " + distance + " // DT " + position_feedback);
    }

    public void stop() {
        drive(0, 0);
    }
}