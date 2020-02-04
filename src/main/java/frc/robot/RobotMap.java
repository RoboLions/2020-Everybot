package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

public class RobotMap {
    public static final int LEFT_FRONT_DRIVE_PORT = 1;
	public static final int LEFT_BACK_DRIVE_PORT = 3;
	public static final int RIGHT_FRONT_DRIVE_PORT = 2;
    public static final int RIGHT_BACK_DRIVE_PORT = 4;

    public static WPI_TalonSRX leftMotorFront = new WPI_TalonSRX(LEFT_FRONT_DRIVE_PORT);
    public static WPI_TalonSRX rightMotorFront = new WPI_TalonSRX(RIGHT_FRONT_DRIVE_PORT);
    public static WPI_TalonSRX leftMotorBack = new WPI_TalonSRX(LEFT_BACK_DRIVE_PORT);
    public static WPI_TalonSRX rightMotorBack = new WPI_TalonSRX(RIGHT_BACK_DRIVE_PORT);

    public static PigeonIMU imu = new PigeonIMU(leftMotorBack);
}