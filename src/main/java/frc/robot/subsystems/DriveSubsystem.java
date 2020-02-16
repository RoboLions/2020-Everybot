package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.RobotMap;
import frc.robot.lib.RoboLionsPID;

public class DriveSubsystem extends SubsystemBase {
    public static final double kMaxSpeed = 3.0; // 3.0; // meters per second
    public static final double kMaxAngularSpeed = 2 * Math.PI; // 2 * Math.PI; // one rotation per second
    private static final double IN_TO_M = .0254;
    
    /* Every Bot Variables */
    private static final int timeoutMs = 10;
    private static final int MOTOR_ENCODER_CODES_PER_REV = 2048; //4096 for CTRE Mag Encoders, 2048 for the Falcons
    private static final double DIAMETER_INCHES = 5.0; // Flex wheels on Everybot
    
	private static final double WHEEL_DIAMETER = DIAMETER_INCHES * IN_TO_M; // in meters
	private static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
	private static final double TICKS_PER_METER = MOTOR_ENCODER_CODES_PER_REV / WHEEL_CIRCUMFERENCE;
    private static final double METERS_PER_TICKS = 1 / TICKS_PER_METER;
    private static final double BOT_WHEEL_TO_WHEEL_DIAMETER = 0.61;//METERS

    //90 degrees /360 = 2*PI*R 
    private static final double HEADING_BOT_DEG_TO_BOT_WHEEL_DISTANCE = (BOT_WHEEL_TO_WHEEL_DIAMETER * Math.PI)/360.0;

    private static final WPI_TalonFX leftMotor = RobotMap.leftDriveMotor;
    private static final WPI_TalonFX rightMotor = RobotMap.rightDriveMotor;

    private final PigeonIMU imu = RobotMap.drive_imu;

    public RoboLionsPID leftForwardPID = new RoboLionsPID();
    public RoboLionsPID rightForwardPID = new RoboLionsPID();
	public RoboLionsPID headingPID = new RoboLionsPID();
    //public RoboLionsPID limelightPID = new RoboLionsPID();
    public RoboLionsPID positionPID = new RoboLionsPID();

    public DriveSubsystem() {
        ZeroYaw();
        resetEncoders();

        leftMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 1);
        leftMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        leftMotor.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms);
        leftMotor.configVelocityMeasurementWindow(16);//1,2,4,8,16,32,64(default)
        leftMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 5, 10);
        
        rightMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 1);
        rightMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        rightMotor.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms);
        rightMotor.configVelocityMeasurementWindow(16);//1,2,4,8,16,32,64(default)
        rightMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 5, 10);

		leftMotor.setNeutralMode(NeutralMode.Coast);
        rightMotor.setNeutralMode(NeutralMode.Coast);
        
        // leftMotor.setInverted(true) --> enable if we determine they are not even

        // Rate Drive PID
        leftForwardPID.initialize2(
            4.5, // Proportional Gain 2 / 7 / ZN 4.2 / 4.2 // 3.5
            20, // Integral Gain 0.0018 / 0.3 / ZN 32.31 / 14
            0, // Derivative Gain ZN 0.1365
            0, // Cage Limit 0.3
            0, // Deadband
            12,// MaxOutput Volts 0.25 //100
            false,//enableCage
            false
        );

        // Rate Drive PID
        rightForwardPID.initialize2(
            3, // Proportional Gain 0.3 / 7 / ZN 4.2 / 4.2 // 3.5
            20, // Integral Gain 0.0018 / 0.1 / ZN 38 / 14 // 20
            0, // Derivative Gain  ZN 0.116
            0, // Cage Limit //0.3
            0, // Deadband
            12,// MaxOutput Volts 0.25 //100
            false,//enableCage
            false
        );

        // Position Command PID for Autonomous and 
        positionPID.initialize2(
            1.35, // Proportional Gain 0.3 / 7 / ZN 4.2 / 4.2 //1.5
            5, // Integral Gain 0.0018 / 0.1 / ZN 38 //20
            0, // Derivative Gain  ZN 0.116
            0.2, // Cage Limit //0.3 //0.1
            0, // Deadband
            1,// MaxOutput Meters/sec 0.25 //100 
            true,//enableCage
            false
        );

        // Heading Command PID for Autonomous and 
        headingPID.initialize2(
            0, // Proportional Gain 0.3 / 7 / ZN 4.2 / 4.2
            0, // Integral Gain 0.0018 / 0.1 / ZN 38
            0, // Derivative Gain  ZN 0.116
            0, // Cage Limit //0.3
            0, // Deadband
            180,// MaxOutput Degrees/sec 0.25 //100 
            false,//enableCage
            false
        );
    }

    // feedforward calculation
    public double calculateNew(double velocity, double acceleration, double ks, double kv, double ka) {
        return ks * Math.signum(velocity) + kv * velocity + ka * acceleration;
    }

    /*****************************************************************************
    * 2/15/20 Use this for anything that has to deal with closed loop rate control
    ******************************************************************************/
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
    
        final double leftFeedforward = calculateNew(leftSpeed, 0, 1.4, 2.6, 0); // ks 1 to 1.5
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

        leftMotor.set(-LVoltagePercentCommand);
        rightMotor.set(RVoltagePercentCommand);

        // m_leftGroup.setVoltage(leftOutput + leftFeedforward);
        // m_rightGroup.setVoltage(rightOutput + rightFeedforward);
        // m_leftGroup.setVoltage(JoystickDrive.throttle);
        // m_rightGroup.setVoltage(JoystickDrive.throttle);

        // System.out.println("Hello World!");
        //System.out.println("LFF " + leftFeedforward + " LPD " + leftOutput + " RFF " + rightFeedforward + " RPD " + rightOutput);
        // There is a + for the error since right encoder velocity was already negative so -- = +
        // System.out.println("Error L - R" + (getLeftEncoderVelocityMetersPerSecond()-getRightEncoderVelocityMetersPerSecond()));
        SmartDashboard.putNumber("Error L - R", (getLeftEncoderVelocityMetersPerSecond()-getRightEncoderVelocityMetersPerSecond()));
        
        SmartDashboard.putNumber("Left Motor Voltage", leftMotor.getMotorOutputVoltage());
        SmartDashboard.putNumber("Right Motor Voltage", rightMotor.getMotorOutputVoltage());
        // System.out.println("L: " + getLeftEncoderVelocityMetersPerSecond() + "/ R:" + getRightEncoderVelocityMetersPerSecond());
        // System.out.println("Left Error: " + (leftSpeed-getLeftEncoderVelocityMetersPerSecond()) + "/ Right Error: " + (getRightEncoderVelocityMetersPerSecond()-rightSpeed));
        // System.out.println("Debug Out  " + rightOutput + " /// " + rightFeedforward + " /// " + JoystickDrive.throttle);
    }

    /*****************************************************************************
    * 2/15/20 Use this for anything that has to deal with closed loop rate control
    ******************************************************************************/
    public void driveWithRotation(double linearTravelSpeed, double rotateSpeed) {
        // input speed is meters per second, input rotation is bot rotation 
        // speed in meters per second
        linearTravelSpeed = (-1*linearTravelSpeed);
        double leftSpeed = (linearTravelSpeed + rotateSpeed);
        double rightSpeed = (linearTravelSpeed - rotateSpeed);
        straightDrive(leftSpeed, rightSpeed);
    }

    @Override
    public void periodic() {

    }

    public void setModeVoltage() {
        leftMotor.set(ControlMode.PercentOutput, 0);
        rightMotor.set(ControlMode.PercentOutput, 0);
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
		return leftMotor.getSelectedSensorPosition();
	}

	public double getRightEncoderPosition() {
		return rightMotor.getSelectedSensorPosition();
	}
	
	public double getLeftEncoderVelocity() {
		return leftMotor.getSelectedSensorVelocity();
	}

	public double getRightEncoderVelocity() {
        return rightMotor.getSelectedSensorVelocity();
    }	
    
    public double getLeftEncoderVelocityMetersPerSecond() {
        //getQuadVelocity is in 100 ms so we have to divide it by 10 to get seconds
        double leftVelocityMPS = (leftMotor.getSelectedSensorVelocity()*10); // /10
        // since getQuadVelocity is in encoder ticks, we have to convert it to meters
        leftVelocityMPS = leftVelocityMPS * METERS_PER_TICKS;
        return (leftVelocityMPS);
    }
    
    public double getRightEncoderVelocityMetersPerSecond() {
        //getQuadVelocity is in 100 ms so we have to divide it by 10 to get seconds
        double rightVelocityMPS = (rightMotor.getSelectedSensorVelocity()*10); // /10
        // since getQuadVelocity is in encoder ticks, we have to convert it to meters
        //Need to have a negative for right velocity since the motors are reversed on the opposite side
        rightVelocityMPS = -rightVelocityMPS * METERS_PER_TICKS;
        return (rightVelocityMPS);
    }

    public double leftDistanceTravelledInMeters() {
        double left_dist = getLeftEncoderPosition() * METERS_PER_TICKS;
        return left_dist;
    }

    public double rightDistanceTravelledInMeters() {
        double right_dist = getRightEncoderPosition() * METERS_PER_TICKS;
        return right_dist;
    }

	public double distanceTravelledinMeters() {
        // left distance is negative because the encoder value on the 
        // left is negative when dev bot is pushed forward 2/15/20
        // Code Tested on Dev Bot, Works on 2/15/20
        double distanceTravelled = (rightDistanceTravelledInMeters() - leftDistanceTravelledInMeters()) / 2;
		return distanceTravelled;
	}

	public void resetEncoders() {
		leftMotor.setSelectedSensorPosition(0);
		rightMotor.setSelectedSensorPosition(0);
    }
    
    // Advanced Drivetrain utilizing PID
    public void driveRoboLionsPID(double throttle, double rotate) {
        //  throttle is in meters per second, rotate is bot rotation speed in meters per second
        driveWithRotation(throttle, rotate);
    }

    // basic, no frills, no PID, no nothing drivetrain
    public static void drive(double throttle, double rotate) {
        leftMotor.set(throttle + rotate);
        rightMotor.set(throttle - rotate);
    }
    
    /*****************************************************************************
    * 2/15/20 Use this for anything that has to deal with closed loop rate control
    ******************************************************************************/
    public void autoDrive(double distance) { // distance is in meters
        // double left_speed; 
        // double right_speed;

        double headingFeedback = getYaw(); // in degrees
        double headingCommand = 0;
        double headingError = headingPID.execute(headingCommand, headingFeedback);
        double headingErrorMeters = HEADING_BOT_DEG_TO_BOT_WHEEL_DISTANCE * headingError;




        double position_feedback = distanceTravelledinMeters();

        // positionError is in meters per second
        double positionError = positionPID.execute(distance, position_feedback);

        // left_speed = output;
        // right_speed = output;

        // straightDrive(left_speed, right_speed);
        // Refer to the rate drive control diagram
        // We modulate our speed of the bot to close out
        // the position error, making it eventually zero
        driveWithRotation(positionError, headingErrorMeters);
        // riveWithRotation(0.5, 0.0);
         System.out.println("Pos " + position_feedback + " PE " + positionError);
        // System.out.println("TD " + distance + " // DT " + position_feedback);
    }

    public void stop() {
        drive(0, 0);
    }
}