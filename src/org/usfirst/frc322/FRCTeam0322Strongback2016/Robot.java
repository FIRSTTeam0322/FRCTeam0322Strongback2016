/* Created Sat Jan 23 12:56:39 EST 2016 */
package org.usfirst.frc322.FRCTeam0322Strongback2016;

import org.strongback.Strongback;
import org.strongback.SwitchReactor;
import org.strongback.components.AngleSensor;
import org.strongback.components.CurrentSensor;
import org.strongback.components.Motor;
import org.strongback.components.ThreeAxisAccelerometer;
import org.strongback.components.VoltageSensor;
import org.strongback.components.ui.ContinuousRange;
import org.strongback.components.ui.FlightStick;
import org.strongback.components.ui.Gamepad;
import org.strongback.drive.TankDrive;
import org.strongback.hardware.Hardware;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.interfaces.Accelerometer.Range;

public class Robot extends IterativeRobot {

	private static final int LEFT_DRIVESTICK_PORT = 0;
	private static final int RIGHT_DRIVESTICK_PORT = 1;
	private static final int MANIPULATOR_STICK_PORT = 2;
	private static final int LF_MOTOR_PORT = 0;
	private static final int LR_MOTOR_PORT = 1;
	private static final int RF_MOTOR_PORT = 3;
	private static final int RR_MOTOR_PORT = 2;
	private static final int LEFT_BALL_SUCK = 4;
	private static final int RIGHT_BALL_SUCK = 5;
	private static final int LEFT_BALL_SHOOT = 6;
	private static final int RIGHT_BALL_SHOOT = 7;
	
	private static final int LEFT_ENCOODER_PORT_A = 0;
	private static final int LEFT_ENCOODER_PORT_B = 1;
	private static final int RIGHT_ENCOODER_PORT_A = 2;
	private static final int RIGHT_ENCOODER_PORT_B = 3;
	private static final double ENCOODER_PULSE_DISTANCE = 1.0;
	
	private static final int AUTON_MODE = 4;
	private static final double AUTON_SPEED = 0.55;
	private static final double AUTON_DISTANCE = 20000.0;
	
	private static final SPI.Port ACCEL_PORT = SPI.Port.kOnboardCS1;
	private static final Range ACCEL_RANGE = Range.k2G;
	private static final SPI.Port GYRO_PORT = SPI.Port.kOnboardCS0;

	private FlightStick leftDriveStick, rightDriveStick;
	private Gamepad manipulatorStick;
	private Motor ballSuckMotor, ballShootMotor;
	
	private TankDrive drivetrain;
	private ContinuousRange leftSpeed, rightSpeed;
	private SwitchReactor ballSuck, ballSpit, stopCollector;
	private SwitchReactor shootBall, shooterReverse, stopShooter;

	private ThreeAxisAccelerometer accel;
	private AngleSensor gyro;
	private AngleSensor leftEncoder, leftEncoderRaw;
	private AngleSensor rightEncoder;
	
	public static CameraServer cameraServer;	
	
	@Override
    public void robotInit() {
    	//Setup drivetrain
    	Motor leftDriveMotors = Motor.compose(Hardware.Motors.talon(LF_MOTOR_PORT),
    											Hardware.Motors.talon(LR_MOTOR_PORT));
    	Motor rightDriveMotors = Motor.compose(Hardware.Motors.talon(RF_MOTOR_PORT),
    											Hardware.Motors.talon(RR_MOTOR_PORT));
    	drivetrain = new TankDrive(leftDriveMotors, rightDriveMotors);
    	
    	//Setup manipulators
    	ballSuckMotor = Motor.compose(Hardware.Motors.talon(LEFT_BALL_SUCK),
    									Hardware.Motors.talon(RIGHT_BALL_SUCK).invert());
    	ballShootMotor = Motor.compose(Hardware.Motors.talon(LEFT_BALL_SHOOT),
    									Hardware.Motors.talon(RIGHT_BALL_SHOOT).invert());

     	//Setup joysticks
    	leftDriveStick = Hardware.HumanInterfaceDevices.logitechAttack3D(LEFT_DRIVESTICK_PORT);
    	rightDriveStick = Hardware.HumanInterfaceDevices.logitechAttack3D(RIGHT_DRIVESTICK_PORT);
    	manipulatorStick = Hardware.HumanInterfaceDevices.xbox360(MANIPULATOR_STICK_PORT);
    	
    	//Setup sensors
    	accel = Hardware.Accelerometers.accelerometer(ACCEL_PORT, ACCEL_RANGE);
    	gyro = Hardware.AngleSensors.gyroscope(GYRO_PORT);
    	leftEncoderRaw = Hardware.AngleSensors.encoder(LEFT_ENCOODER_PORT_A, LEFT_ENCOODER_PORT_B, ENCOODER_PULSE_DISTANCE);
    	leftEncoder = AngleSensor.invert(leftEncoderRaw);
    	rightEncoder = Hardware.AngleSensors.encoder(RIGHT_ENCOODER_PORT_A, RIGHT_ENCOODER_PORT_B, ENCOODER_PULSE_DISTANCE);
    	VoltageSensor battery = Hardware.powerPanel().getVoltageSensor();
    	CurrentSensor current = Hardware.powerPanel().getTotalCurrentSensor();
    	
    	//Setup drivetrain variables
    	ContinuousRange sensitivity = leftDriveStick.getAxis(2).invert().map(t -> (t + 1.0) / 2.0);
    	leftSpeed = leftDriveStick.getPitch().scale(sensitivity::read);
    	rightSpeed = rightDriveStick.getPitch().scale(sensitivity::read);
    	
    	//Setup Switches
    	ballSuck = Strongback.switchReactor();
    	ballSpit = Strongback.switchReactor();
    	stopCollector = Strongback.switchReactor();
    	
    	shootBall = Strongback.switchReactor();
    	shooterReverse = Strongback.switchReactor();
    	stopShooter = Strongback.switchReactor();
    	
    	//Setup Camera
    	cameraServer = CameraServer.getInstance();
        cameraServer.setQuality(25);
        cameraServer.startAutomaticCapture("cam0");
        
        Strongback.dataRecorder()
        			.register("Battery Volts", 1000, battery::getVoltage)
        			.register("Current load", 1000, current::getCurrent)
        			.register("Left Motors", leftDriveMotors)
        			.register("Right Motors", rightDriveMotors)
        			.register("LeftDriveStick", 1000, leftSpeed::read)
        			.register("RightDriveStick", 1000, rightSpeed::read)
        			.register("Drive Sensitivity", 1000, sensitivity::read)
        			.register("Gyro", 1000, gyro::getAngle)
        			.register("X-Accel", 1000, accel.getXDirection()::getAcceleration)
        			.register("Y-Accel", 1000, accel.getYDirection()::getAcceleration)
        			.register("Z-Accel", 1000, accel.getZDirection()::getAcceleration)
        			.register("Left Encoder", 1000, leftEncoder::getAngle)
					.register("Right Encoder", 1000, rightEncoder::getAngle);
    	
    	Strongback.configure().recordNoEvents().recordDataToFile("FRCTeam0322Strongback2016-").initialize();
    }

	@Override
    public void autonomousInit() {
        // Start Strongback functions ...
        Strongback.start();
    }
    
	@Override
    public void autonomousPeriodic() {
    	switch(AUTON_MODE) {
    	case 0: Strongback.submit(new DoNothing());
    		break;
    	case 1:
    		if (Math.abs(leftEncoder.getAngle()) < AUTON_DISTANCE ||
    				Math.abs(rightEncoder.getAngle()) < AUTON_DISTANCE) {
    			Strongback.submit(new DriveForward(drivetrain, AUTON_SPEED));
    		}
    		break;
    	case 2:
    		if (Math.abs(leftEncoder.getAngle()) < AUTON_DISTANCE ||
    				Math.abs(rightEncoder.getAngle()) < AUTON_DISTANCE) {
        		Strongback.submit(new DriveBackward(drivetrain, AUTON_SPEED));
        	}
    		break;
    	case 3:
    		Strongback.submit(new DriveForwardAndBack(drivetrain, AUTON_SPEED));
    		break;
    	case 4:
    		Strongback.submit(new DriveBackwardAndFore(drivetrain, AUTON_SPEED));
    		break;
    	default:
    		Strongback.submit(new DoNothing());
    		break;
    	}
    	
    	System.out.println("Gyro Angle " + gyro.getAngle());
    	System.out.println();
    	System.out.println("X-Axis " + accel.getXDirection().getAcceleration());
    	System.out.println("Y-Axis " + accel.getYDirection().getAcceleration());
    	System.out.println("Z-Axis " + accel.getZDirection().getAcceleration());
    	System.out.println();
    	System.out.println("Left Distance " + leftEncoder.getAngle());
    	System.out.println("Right Distance " + rightEncoder.getAngle());
    	System.out.println();
    	System.out.println();
    }
    
	@Override
    public void teleopInit() {
        // Restart Strongback functions ...
        Strongback.restart();
    }
	
	@Override
    public void teleopPeriodic() {
    	//This line runs the drivetrain
    	drivetrain.tank(leftSpeed.read(), rightSpeed.read());

    	//This section handles the collector
    	ballSuck.onTriggered(manipulatorStick.getA(), ()->Strongback.submit(new SuckBall(ballSuckMotor)));
    	ballSuck.onUntriggered(manipulatorStick.getA(), ()->Strongback.submit(new StopCollector(ballSuckMotor)));

    	ballSpit.onTriggered(manipulatorStick.getB(), ()->Strongback.submit(new SpitBall(ballSuckMotor)));
    	ballSpit.onUntriggered(manipulatorStick.getB(), ()->Strongback.submit(new StopCollector(ballSuckMotor)));
    	
    	stopCollector.onTriggered(manipulatorStick.getLeftBumper(), ()->Strongback.submit(new StopCollector(ballSuckMotor)));

    	//This secton handles the shooter
    	shootBall.onTriggered(manipulatorStick.getX(), ()->Strongback.submit(new ShootBall(ballShootMotor)));
    	shootBall.onUntriggered(manipulatorStick.getX(), ()->Strongback.submit(new StopShooter(ballShootMotor)));
    	
    	shooterReverse.onTriggered(manipulatorStick.getY(), ()->Strongback.submit(new ReverseShooter(ballShootMotor)));
    	shooterReverse.onUntriggered(manipulatorStick.getY(), ()->Strongback.submit(new StopShooter(ballShootMotor)));

    	stopShooter.onTriggered(manipulatorStick.getRightBumper(), ()->Strongback.submit(new StopShooter(ballShootMotor)));
    	
    	System.out.println("Gyro Angle " + gyro.getAngle());
    	System.out.println();
    	System.out.println("X-Axis " + accel.getXDirection().getAcceleration());
    	System.out.println("Y-Axis " + accel.getYDirection().getAcceleration());
    	System.out.println("Z-Axis " + accel.getZDirection().getAcceleration());
    	System.out.println();
    	System.out.println("Left Distance " + leftEncoder.getAngle());
    	System.out.println("Right Distance " + rightEncoder.getAngle());
    	System.out.println();
    	System.out.println();
    }

	@Override
    public void disabledInit() {
    	drivetrain.stop();
        // Tell Strongback that the robot is disabled so it can flush and kill commands.
        Strongback.disable();
    }
    
	@Override
    public void disabledPeriodic() {
    }
}
