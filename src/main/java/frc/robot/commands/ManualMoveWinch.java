/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import frc.robot.subsystems.WinchSubsystem;

public class ManualMoveWinch extends CommandBase {

  private final WinchSubsystem winchSubsystem;
  public static WPI_TalonSRX winchMotor = RobotMap.winchMotor;

  private final static XboxController manipulatorController = RobotContainer.driverController;
  public static double mLeftTriggerValue = RobotContainer.manipulatorLeftTriggerValue;
  public static double mRightTriggerValue = RobotContainer.manipulatorRightTriggerValue;
  
  public static double WINCH_SCALER = RobotContainer.winchSubsystem.WINCH_POWER; // TODO test value (WINCH_POWER is in winch subsystem)
  public static final double STOP_POWER = 0.0;

  public ManualMoveWinch(WinchSubsystem winch) {
    // Use addRequirements() here to declare subsystem dependencies.
    winchSubsystem = winch;
    addRequirements(winchSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /* We should tell the drivers to use LT to winch up however
    * I belive it is wise to have RT to go the opposite direction
    * so that we can reset the winch back to an unwrapped position
    */
    if(mLeftTriggerValue > 0.25 && mRightTriggerValue < 0.25) {
      double WINCH_UP_POWER = (mLeftTriggerValue*WINCH_SCALER);
      winchMotor.set(WINCH_UP_POWER);
    } else if(mRightTriggerValue > 0.25  && mLeftTriggerValue < 0.25) {
      double WINCH_DOWN_POWER = -(mRightTriggerValue*WINCH_SCALER);
      winchMotor.set(WINCH_DOWN_POWER);
    } else {
      winchMotor.set(STOP_POWER);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
