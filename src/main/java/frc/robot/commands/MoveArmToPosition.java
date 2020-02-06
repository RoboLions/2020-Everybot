package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

public class MoveArmToPosition extends PIDCommand {

  public MoveArmToPosition(double targetAngleDegrees, ArmSubsystem arm) {
    super(
        new PIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD),
        // Close loop on heading
        arm::getArmAngle,
        // Set reference to target
        targetAngleDegrees,
        // Pipe output to turn the arm
        output -> arm.moveArm(output),
        // Require the arm subsystem
        arm);

    // Set the controller to be continuous (because it is an angle controller)
    getController().enableContinuousInput(-180, 180);
    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    getController()
        .setTolerance(ArmConstants.TURN_TOLERANCE, ArmConstants.SPEED_TOLERANCE);
  }

  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    return getController().atSetpoint();
  }
}