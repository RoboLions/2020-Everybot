/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;

public class WinchSubsystem extends SubsystemBase {

  public final double WINCH_POWER = 0.2; // TODO test value 
  public static final double STOP_POWER = 0.0;

  WPI_TalonSRX winchMotor = RobotMap.winchMotor;

  public WinchSubsystem() {
    winchMotor.setNeutralMode(NeutralMode.Brake);
  }

  public void winchUp() {
    winchMotor.set(WINCH_POWER);
  }

  public void winchDown() {
      RobotMap.armMotor.set(-WINCH_POWER);
  }

  public void stop() {
    winchMotor.set(STOP_POWER);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
