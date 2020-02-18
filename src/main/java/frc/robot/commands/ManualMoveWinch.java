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

  private final static XboxController driverController = RobotContainer.driverController;
  
  public static final double POWER = 0.2; // TODO tune value to proper
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
    double winchPower;

    // left is the correct direction for winching up
    boolean leftTrigger = (driverController.getTriggerAxis(Hand.kLeft) > 0.25); 
    // boolean rightTrigger = (driverController.getTriggerAxis(Hand.kRight) > 0.25);

    if(leftTrigger) {
      winchPower = POWER;
    } 
    /*else if(rightTrigger) {
      winchPower = -POWER;
    }*/ 
    else {
      winchPower = 0;
    }

    winchSubsystem.setWinchPower(winchPower);
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