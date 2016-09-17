package org.usfirst.frc322.FRCTeam0322Strongback2016;

import org.strongback.command.*;
import org.strongback.drive.TankDrive;

public class DriveBackward extends Command {
	private final TankDrive drivetrain;
	private final double speed, rightSpeed;
	
	public DriveBackward(TankDrive drivetrain, double speed) {
		super(drivetrain);
		this.drivetrain = drivetrain;
		this.speed = speed;
		this.rightSpeed = (speed * 0.8);
	}
	
	@Override
	public boolean execute() {
		this.drivetrain.tank(speed, rightSpeed);
		return true;
	}
}
