package org.usfirst.frc322.FRCTeam0322Strongback2016;

import org.strongback.command.*;
import org.strongback.components.Motor;
import org.strongback.components.Switch;
import org.strongback.components.ui.ContinuousRange;

public class LiftExtend extends Command {
	private final Motor motor;
	private final Switch lowerLimit, upperLimit;
	private final ContinuousRange speed;
	
	public LiftExtend(Motor motor, Switch lowerLimit, Switch upperLimit, ContinuousRange speed) {
		super(motor);
		this.motor = motor;
		this.lowerLimit = lowerLimit;
		this.upperLimit = upperLimit;
		this.speed = speed;
	}
	
	@Override
	public boolean execute() {
		this.motor.setSpeed(speed.read());
		if(speed.read() >= 0.05){
			return this.upperLimit.isTriggered();
		}
		else if(speed.read() <= -0.05){
			return this.lowerLimit.isTriggered();
		}
		else
			return true;
	}
}
