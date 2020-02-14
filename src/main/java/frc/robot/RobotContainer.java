package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import frc.robot.Constants.OIConstants;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.commands.ManualMoveArm;
import frc.robot.commands.ManualMoveClimb;
import frc.robot.commands.ManualMoveWinch;
import frc.robot.commands.JoystickDrive;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.WinchSubsystem;
import frc.robot.commands.AutoMove;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems
    public static DriveSubsystem driveSubsystem = new DriveSubsystem();
    public static IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    public static WinchSubsystem winchSubsystem = new WinchSubsystem();
    public static ArmSubsystem armSubsystem = new ArmSubsystem();
    public static ClimberSubsystem climberSubsystem = new ClimberSubsystem();
    public static DifferentialDriveWheelSpeeds difWheelSpeeds = new DifferentialDriveWheelSpeeds();

    // The driver's controller
    public static XboxController driverController = new XboxController(OIConstants.DRIVER_CONTROLLER_PORT);
    public static XboxController manipulatorController = new XboxController(OIConstants.MANIPULATOR_CONTROLLER_PORT);
    
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

        armSubsystem.setDefaultCommand(
            new ManualMoveArm(armSubsystem)
        );

        climberSubsystem.setDefaultCommand(
            new ManualMoveClimb(climberSubsystem)
        );

        winchSubsystem.setDefaultCommand(
            new ManualMoveWinch(winchSubsystem)
        );
    }

    /**
     * Use this method to define your button->command mappings.  Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
     * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /*** MANIPULATOR CONTROLLER***/
        // ball intake
        new JoystickButton(manipulatorController, Button.kA.value).whileHeld(
            new InstantCommand(intakeSubsystem::intakeBalls, intakeSubsystem)
        );
        // ball outtake
        new JoystickButton(manipulatorController, Button.kY.value).whileHeld(
            new InstantCommand(intakeSubsystem::outtakeBalls, intakeSubsystem)
        );

        /*** DRIVER CONTROLLER***/
        // testing auto
        new JoystickButton(driverController, Button.kA.value).whenPressed(
            new AutoMove(driveSubsystem, 1.0)
        );

        // COMPLETED climber up (LB) and climber down (RB)
        // COMPLETED Winch pull in (LT) and Winch let out (RT)
    }  
}
