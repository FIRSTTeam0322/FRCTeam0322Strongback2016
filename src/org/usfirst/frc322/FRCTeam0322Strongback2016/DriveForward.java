package org.usfirst.frc322.FRCTeam0322Strongback2016;

import org.strongback.command.*;
import org.strongback.drive.TankDrive;

public class DriveForward extends Command {
	private final TankDrive drivetrain;
	private static final double SPEED;
	
	public DriveForward(TankDrive drivetrain) {
		super(drivetrain);
		this.drivetrain = drivetrain;
		SPEED = 0.75;
	}
	
	public boolean execute() {
		this.drivetrain.tank(-SPEED, -SPEED);
		return true;
	}
}
