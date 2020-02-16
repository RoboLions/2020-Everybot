package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class RobotMap {
    //public static final int LEFT_FRONT_DRIVE_PORT = 1;
	//public static final int LEFT_BACK_DRIVE_PORT = 3;
	//public static final int RIGHT_FRONT_DRIVE_PORT = 2;
    //public static final int RIGHT_BACK_DRIVE_PORT = 4;

    public static final int INTAKE_PORT = 10; // placeholder
    public static final int ARM_PORT = 13; // placeholder
    public static final int ARM_IMU_PORT = 14; // placeholder
    public static final int CLIMBER_PORT = 11; // placeholder
    public static final int WINCH_PORT = 12; // placeholder
    
    /************************************************************************************************************/

    //public static WPI_TalonSRX leftMotorFront = new WPI_TalonSRX(LEFT_FRONT_DRIVE_PORT);
    //public static WPI_TalonSRX rightMotorFront = new WPI_TalonSRX(RIGHT_FRONT_DRIVE_PORT);
    //public static WPI_TalonSRX leftMotorBack = new WPI_TalonSRX(LEFT_BACK_DRIVE_PORT);
    //public static WPI_TalonSRX rightMotorBack = new WPI_TalonSRX(RIGHT_BACK_DRIVE_PORT);

    //public static DifferentialDrive robotDrive = new DifferentialDrive(leftMotorFront, rightMotorFront);
    
    public static WPI_TalonSRX intakeMotor = new WPI_TalonSRX(INTAKE_PORT);

    public static WPI_TalonSRX armMotor = new WPI_TalonSRX(ARM_PORT);

    public static PigeonIMU arm_imu = new PigeonIMU(ARM_IMU_PORT);

    public static WPI_TalonSRX climberMotor = new WPI_TalonSRX(CLIMBER_PORT);

    public static PigeonIMU drive_imu = new PigeonIMU(climberMotor);

    public static WPI_TalonSRX winchMotor = new WPI_TalonSRX(WINCH_PORT);
}