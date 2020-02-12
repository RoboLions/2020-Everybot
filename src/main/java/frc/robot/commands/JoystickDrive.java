package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;

public class JoystickDrive extends CommandBase {
    private final DriveSubsystem driveSubsystem;
    private final static XboxController driverController = RobotContainer.driverController;
    public static double throttle = driverController.getY(Hand.kLeft);
    public double rotate = driverController.getX(Hand.kRight);

    // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
    private final SlewRateLimiter m_speedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

    public JoystickDrive(DriveSubsystem drivetrain) {
        driveSubsystem = drivetrain;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        driveSubsystem.stop();
    }

    @Override
    public void execute() {
        
        /*
        // PID Testing Joystick Inputs
        if (driverController.getY(Hand.kLeft) > 0.2) 
        {
            throttle = 2;
        } 
        else if (driverController.getY(Hand.kLeft) < -0.2) {
            throttle = -2;
        }
        else {
            throttle = 0.0;
        }
        rotate = 0.0;
        */

        throttle = driverController.getY(Hand.kLeft);
        rotate = driverController.getX(Hand.kRight);
        
        if(throttle > 1) {
            throttle = 1;
        } else if(throttle < -1) {
            throttle = -1;
        }

        // this is the rate curve that we calculated to get the joystick feeling really nice
        double joystick_target = throttle;//1.458*Math.pow(throttle, 4)-2.0234*Math.pow(throttle, 3)+0.7049*Math.pow(throttle, 2)+0.1116*throttle-0.0203;

        //driveSubsystem.drive(throttle, rotate);
        driveSubsystem.driveRoboLionsPID(joystick_target, rotate);
        //System.out.println("JD " + throttle);
        /*
        // Get the x speed. We are inverting this because Xbox controllers return
        // negative values when we push forward.
        final var xSpeed = -m_speedLimiter.calculate(throttle) * DriveSubsystem.kMaxSpeed;

        // Get the rate of angular rotation. We are inverting this because we want a
        // positive value when we pull to the left (remember, CCW is positive in
        // mathematics). Xbox controllers return positive values when you pull to
        // the right by default.
        final var rot = -m_rotLimiter.calculate(rotate) * DriveSubsystem.kMaxAngularSpeed;
        */

        // driveSubsystem.drive(xSpeed, rot);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}