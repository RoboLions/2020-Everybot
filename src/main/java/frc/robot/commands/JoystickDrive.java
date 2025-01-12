package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

public class JoystickDrive extends CommandBase {
    private final DriveSubsystem driveSubsystem;
    private final static XboxController driverController = RobotContainer.driverController;
    public static double _throttle = driverController.getY(Hand.kLeft);
    public double _rotate = driverController.getX(Hand.kRight);

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
    /** 
    double throttle_sign;

    throttle_sign = throttle / (Math.abs(throttle));

    if((throttle > 0 && throttle < 0.25) || (throttle < 0 && throttle > -0.25)) {
        throttle = 0;
    }

    if((rotate > 0 && rotate < 0.25) || (rotate < 0 && rotate > -0.25)) {
        rotate = 0;
    }

    //limit the throttle to 1.0
    if (throttle > 1) {
         throttle = 1;
    } else if (throttle < -1) {
         throttle = -1;
    }
    
    //run the remapping function, convert the throttle input to speed output, can only accept a + input, -, input
    //causes the function to "blow up"
    //y = 1.458x4 - 2.0234x3 + 0.7049x2 + 0.1116x - 0.0203
    throttle = Math.abs(throttle);

    double remapped_rate =
           (1.458*throttle*throttle*throttle*throttle
           - 2.0234*throttle*throttle*throttle
           + 0.7049*throttle*throttle
           + 0.1116*throttle
           - 0.0203);
 
    remapped_rate = remapped_rate * throttle_sign;

    if (throttle_sign > 0.0) {
         remapped_rate += 0.02;
    } else {
         remapped_rate -= 0.02;
    }

    double speed_multiplier = 4.0;


    double joystick_output = speed_multiplier  * remapped_rate;
    */
    
    //driveSubsystem.driveRoboLionsPID(joystick_output, rotate);
    //driveSubsystem.driveRoboLionsPID(joystick_output, 0.0);
    // driveSubsystem.driveRoboLionsPID(remapped_rate, rotate);
        
        double throttle = driverController.getY(Hand.kLeft);
        double rotate = driverController.getX(Hand.kRight);
        
        // SlewRateLimiter t_limiter = new SlewRateLimiter(0.5);
        // SlewRateLimiter r_lLimiter = new SlewRateLimiter(0.5); 

       //throttle *= 3;
       //rotate *= 3;
       
       /*
        if(throttle > 1) {
            throttle = 1;
        } else if(throttle < -1) {
            throttle = -1;
        }
        */

        if((throttle > 0 && throttle < 0.25) || (throttle < 0 && throttle > -0.25)) {
            throttle = 0;
        } else {
            throttle = (Math.tan(.465 * (throttle * Math.PI))) / 3;
        }

        if((rotate > 0 && rotate < 0.25) || (rotate < 0 && rotate > -0.25)) {
            rotate = 0;
        }

        // rotation speed scaler
        rotate = 2*rotate;

        // double new_throttle = t_limiter.calculate(throttle);
        // double new_rotate = r_lLimiter.calculate(rotate);

        // The speed limit is a multiple of throttle

        // Slow Mode
        if (driverController.getTriggerAxis(Hand.kRight) > 0.25) {
            throttle = Math.signum(throttle) * 0.75;
        }
        // Fast Mode
        else if (driverController.getAButton()) {
            throttle = (throttle*1.1);
        }
        // Normal Driving Speed
        else {
            throttle = (throttle*0.8);
        }
        

        driveSubsystem.driveRoboLionsPID(-throttle, rotate);
        //driveSubsystem.driveRoboLionsPID(-throttle*2.5, rotate);
        
        // this is the rate curve that we calculated to get the joystick feeling really nice
        /*
        double joystick_target = 3*throttle;//1.458*Math.pow(throttle, 4)-2.0234*Math.pow(throttle, 3)+0.7049*Math.pow(throttle, 2)+0.1116*throttle-0.0203;

        //driveSubsystem.drive(throttle, rotate);
        driveSubsystem.driveRoboLionsPID(joystick_target, rotate);
        */
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