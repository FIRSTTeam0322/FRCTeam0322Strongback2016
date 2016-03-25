package org.usfirst.frc322.FRCTeam0322Strongback2016;

import org.strongback.command.*;
import org.strongback.components.Motor;
import org.strongback.components.Switch;

public class LiftRaise extends Command {
	private final Motor motor;
	private final Switch upperLimit;
	
	public LiftRaise(Motor motor, Switch upperLimit) {
		super(motor);
		this.motor = motor;
		this.upperLimit = upperLimit;
	}
	
	@Override
	public boolean execute() {
		this.motor.setSpeed(1.0);
		return this.upperLimit.isTriggered();
	}
}
