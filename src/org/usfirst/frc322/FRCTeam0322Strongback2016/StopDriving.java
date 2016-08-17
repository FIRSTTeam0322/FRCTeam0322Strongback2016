package org.usfirst.frc322.FRCTeam0322Strongback2016;

import org.strongback.command.*;
import org.strongback.drive.TankDrive;

public class StopDriving extends Command {
	private final TankDrive drivetrain;
	private final double speed;
	
	public StopDriving(TankDrive drivetrain) {
		super(drivetrain);
		this.drivetrain = drivetrain;
		this.speed = 0.0;
	}
	
	@Override
	public boolean execute() {
		this.drivetrain.tank(speed, speed);
		return true;
	}
}
