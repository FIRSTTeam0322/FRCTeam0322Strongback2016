package org.usfirst.frc322.FRCTeam0322Strongback2016;

import org.strongback.command.*;
import org.strongback.components.Motor;

public class SpitBall extends Command {
	private final Motor motor;
	
	public SpitBall(Motor motor) {
		super(motor);
		this.motor = motor;
	}
	
	public boolean execute() {
		this.motor.setSpeed(-1.0);
		return true;
	}
}
