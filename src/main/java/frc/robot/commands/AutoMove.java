/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

/**
 *
 */
public class AutoMove extends CommandBase {
    private final DriveSubsystem drivesubsystem;
    private double start_dist_meters;
    private double target_distance;
    
	public AutoMove(final DriveSubsystem subsystem, double distance) {
		drivesubsystem = subsystem;
        addRequirements(subsystem);
        start_dist_meters = drivesubsystem.distanceTravelledinMeters();
        target_distance = distance;
	}

	@Override
	public void initialize() {
		
	}

	@Override
	public void execute() {
        drivesubsystem.autoDrive(target_distance);
        System.out.println("AUTO WORKS");
	}

	@Override
	public boolean isFinished() {
        System.out.println("AUTO FINISHED");
        double distance_driven = drivesubsystem.distanceTravelledinMeters() - start_dist_meters;
        double positionError = Math.abs(target_distance - distance_driven);
        return(positionError < 0.001);
	}
}	
