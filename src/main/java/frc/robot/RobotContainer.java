package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.OIConstants;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.JoystickDrive;
import frc.robot.subsystems.DriveSubsystem;

import static edu.wpi.first.wpilibj.XboxController.Button;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems
    private final DriveSubsystem driveSubsystem = new DriveSubsystem();

    // The driver's controller
    public static XboxController driverController = new XboxController(OIConstants.DRIVER_CONTROLLER_PORT);
    public static XboxController manipulatorController = new XboxController(OIConstants.MANIPULATOR_CONTROLLER_PORT);

    public static double driverYStickLeft = manipulatorController.getY(Hand.kLeft);
    public static double driverXStickRight = manipulatorController.getX(Hand.kRight);
    public static double driverLeftTriggerValue = driverController.getTriggerAxis(Hand.kLeft);
    public static double driverRightTriggerValue = driverController.getTriggerAxis(Hand.kRight);
    public static double leftAxisValue = driverController.getRawAxis(OIConstants.LEFT_AXIS);
  
    public static double manipulatorYStickLeft = manipulatorController.getY(Hand.kLeft);
    public static double manipulatorLeftTriggerValue = manipulatorController.getTriggerAxis(Hand.kLeft);
    public static double manipulatorRightTriggerValue = manipulatorController.getTriggerAxis(Hand.kRight);
    
    /**
     * The container for the robot.  Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();

        driveSubsystem.setDefaultCommand(
            new JoystickDrive(driveSubsystem)
        );
    }

    /**
     * Use this method to define your button->command mappings.  Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
     * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {

    }
}
