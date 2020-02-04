package frc.robot;

import edu.wpi.first.wpilibj.util.Color;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class OIConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int MANIPULATOR_CONTROLLER_PORT = 1;
    public static int LEFT_AXIS = 0;
    public static int RIGHT_AXIS = 1;
  }
  public static final class LimelightConstants 
  {
    //modes for limelight led light
    public static double FORCE_OFF = 1;
    public static double FORCE_BLINK = 2;
    public static double FORCE_ON = 3;

    //modes for limelight camera 
    public static double VISION_PROCESSOR = 0;
    public static double DRIVER_CAMERA = 1;
  }

  public static final class ShooterConstants 
  {
    // all the below numbers are placeholders that we need real values for
    public static final int SHOOTER_TOP_PORT = 400; // placeholder
    public static final int SHOOTER_BOTTOM_PORT = 500; // placeholder

    public static final double VOLTS = 1;
    public static final double VOLT_SEC_PER_ROT = 2;
    public static final double MOTOR_PIDF_PROFILE = 0;
    public static final double MOTOR_GAIN_F = 3;
    public static final double MOTOR_GAIN_P = 4;
    public static final double MOTOR_GAIN_I = 5;	
    public static final double MOTOR_GAIN_D = 6;
    public static final double TOLERANCE_RPM = 7;
    public static final double TARGET_RPM = 8;
    public static final double ENCODER_DISTANCE_PER_PULSE = 9;
    public static final double MOTOR_ENCODER_TICKS_PER_REV = 10;
    public static final double REVS_PER_TICK = 1/MOTOR_ENCODER_TICKS_PER_REV;
  }

  public static final class ColorWheelConstants 
  {
    public static final double WHEEL_POWER = 0.5;

    public static final Color RED = new Color(0.465, 0.3803, 0.1563);
    public static final Color YELLOW = new Color(0.32, 0.5463, 0.132);
    public static final Color GREEN = new Color(0.186, 0.52767, 0.3553);
    public static final Color BLUE = new Color(0.1403, 0.4453, 0.409);

    public static final double CONTROL_PANEL_CIRCUMFERENCE = 100.530964915; //pi*diameter
    public static final double SPINNER_CIRCUMFERENCE = 0.0; //placeholder 
    public static final double TOTAL_ROTATIONS = (CONTROL_PANEL_CIRCUMFERENCE/SPINNER_CIRCUMFERENCE)*4;
    public static final double ENCODER_TICKS_PR = 1440; //placeholder 
    public static final double TOTAL_ENCODER_TICKS = TOTAL_ROTATIONS * ENCODER_TICKS_PR;
    public static double COUNT; //individual encoder ticks
  }
}