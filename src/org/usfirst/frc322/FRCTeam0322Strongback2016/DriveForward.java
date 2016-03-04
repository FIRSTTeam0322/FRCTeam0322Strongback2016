package org.usfirst.frc322.FRCTeam0322Strongback2016;

import org.strongback.command.*;
import org.strongback.drive.TankDrive;

public class DriveForward extends Command {
	private final TankDrive drivetrain;
	
	public DriveForward(TankDrive drivetrain) {
		super(drivetrain);
		this.drivetrain = drivetrain;
	}
	
	public boolean execute() {
		this.drivetrain.tank(0.75, 0.75);
		return true;
	}
}
