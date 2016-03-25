package org.usfirst.frc322.FRCTeam0322Strongback2016;

import org.strongback.command.*;
import org.strongback.components.Motor;
import org.strongback.components.Switch;

public class LiftLower extends Command {
	private final Motor motor;
	private final Switch lowerLimit;
	
	public LiftLower(Motor motor, Switch lowerLimit) {
		super(motor);
		this.motor = motor;
		this.lowerLimit = lowerLimit;
	}
	
	@Override
	public boolean execute() {
		this.motor.setSpeed(-1.0);
		return this.lowerLimit.isTriggered();
	}
}
