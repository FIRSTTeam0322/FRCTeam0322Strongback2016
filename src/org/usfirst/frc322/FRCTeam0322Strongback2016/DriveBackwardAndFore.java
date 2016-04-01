package org.usfirst.frc322.FRCTeam0322Strongback2016;

import org.strongback.command.*;
import org.strongback.components.AngleSensor;
import org.strongback.drive.TankDrive;

public class DriveBackwardAndFore extends Command {
	private final TankDrive drivetrain;
	private final double speed;
	private final double distance;
	private final AngleSensor leftEncoder, rightEncoder;
	private boolean stepOneComplete, stepTwoComplete, stepThreeComplete;

	public DriveBackwardAndFore(TankDrive drivetrain, double speed, double distance,
									AngleSensor leftEncoder, AngleSensor rightEncoder) {
		super(drivetrain);
		this.drivetrain = drivetrain;
		this.speed = speed;
		this.distance = distance;
		this.leftEncoder = leftEncoder;
		this.rightEncoder = rightEncoder; 

	}
	
	@Override
	public boolean execute() {
		if ((Math.abs(leftEncoder.getAngle()) < distance ||
				Math.abs(rightEncoder.getAngle()) < distance) && !stepOneComplete && !stepTwoComplete) {
    		drivetrain.tank(speed, speed);
    	} else if (((Math.abs(leftEncoder.getAngle()) <= 0 ||
				Math.abs(rightEncoder.getAngle()) <= 0) || stepOneComplete) && !stepTwoComplete) {
    		stepOneComplete = true;
    		drivetrain.tank(-speed, -speed);
		} else if (((Math.abs(leftEncoder.getAngle()) < distance ||
				Math.abs(rightEncoder.getAngle()) < distance) || stepTwoComplete) && !stepThreeComplete) {
			stepTwoComplete = true;
			drivetrain.tank(speed, speed);
		} else if (stepTwoComplete && !stepThreeComplete) {
				stepThreeComplete = true;	
				drivetrain.tank(0, 0);
		}
		return stepThreeComplete;
	}
}
