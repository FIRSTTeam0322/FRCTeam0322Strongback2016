package org.usfirst.frc322.FRCTeam0322Strongback2016;

import org.strongback.command.*;
import org.strongback.components.Motor;

public class LiftRetract extends Command {
	private final Motor motor;
	private final double speed;
	
	public LiftRetract(Motor motor, double speed) {
		super(motor);
		this.motor = motor;
		this.speed = speed;
	}
	
	@Override
	public boolean execute() {
		this.motor.setSpeed(this.speed);
		return true;
	}
}
