package org.usfirst.frc322.FRCTeam0322Strongback2016;

import org.strongback.command.*;
import org.strongback.components.Motor;

public class StopShooter extends Command {
	private final Motor motor;
	
	public StopShooter(Motor motor) {
		super(motor);
		this.motor = motor;
	}
	
	@Override
	public boolean execute() {
		this.motor.setSpeed(0.0);
		return true;
	}
}
