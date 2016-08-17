package org.usfirst.frc322.FRCTeam0322Strongback2016;

import org.strongback.command.*;
import org.strongback.drive.TankDrive;

public class DriveForwardAndBack extends CommandGroup {
	private final TankDrive drivetrain;
	private final double speed;

	public DriveForwardAndBack(TankDrive drivetrain, double speed) {
		this.drivetrain = drivetrain;
		this.speed = speed;
		sequentially(Command.create(5.0, ()->{			
						this.drivetrain.tank(-this.speed, -this.speed);
						boolean complete = false;
						return complete;}),
					Command.create(5.0, ()->{			
						this.drivetrain.tank(this.speed, this.speed);
						boolean complete = false;
						return complete;}),
					new StopDriving(drivetrain));
	}
}
