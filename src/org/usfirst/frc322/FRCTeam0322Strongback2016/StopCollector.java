package org.usfirst.frc322.FRCTeam0322Strongback2016;

import org.strongback.command.*;
import org.strongback.components.Motor;

public class StopCollector extends Command {
	private final Motor motor;
	
	public StopCollector(Motor motor) {
		super(motor);
		this.motor = motor;
	}
	
	public boolean execute() {
		this.motor.setSpeed(0.0);
		return true;
	}
}
