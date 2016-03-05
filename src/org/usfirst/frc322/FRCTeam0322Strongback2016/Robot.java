/* Created Sat Jan 23 12:56:39 EST 2016 */
package org.usfirst.frc322.FRCTeam0322Strongback2016;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.interfaces.Accelerometer.Range;

import java.util.concurrent.TimeUnit;
import java.lang.Math;

import org.strongback.Strongback;
import org.strongback.SwitchReactor;
import org.strongback.components.Motor;
import org.strongback.components.ui.ContinuousRange;
import org.strongback.components.ui.FlightStick;
import org.strongback.components.ui.Gamepad;
import org.strongback.components.ThreeAxisAccelerometer;
import org.strongback.components.AngleSensor;
import org.strongback.drive.TankDrive;
import org.strongback.hardware.Hardware;

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
	
	private static final int AUTON_MODE = 2;
	private static final double AUTON_SPEED = 0.55;
	private static final int AUTON_DISTANCE = 20000;
	
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
	private AngleSensor leftEncoder, leftEncoderAdjusted;
	private AngleSensor rightEncoder;
	
	public static CameraServer cameraServer;	
	
    public void robotInit() {
    	//Setup drivetrain
    	Motor leftDriveMotor = Motor.compose(Hardware.Motors.talon(LF_MOTOR_PORT),
    											Hardware.Motors.talon(LR_MOTOR_PORT));
    	//LiveWindow.addActuator("Left Front Motor", LF_MOTOR_PORT, (Talon) Hardware.Motors.talon(LF_MOTOR_PORT));
    	//LiveWindow.addActuator("Left Rear Motor", LR_MOTOR_PORT, (Talon) Hardware.Motors.talon(LR_MOTOR_PORT));
    	Motor rightDriveMotor = Motor.compose(Hardware.Motors.talon(RF_MOTOR_PORT),
    											Hardware.Motors.talon(RR_MOTOR_PORT));
    	//LiveWindow.addActuator("Right Front Motor", RF_MOTOR_PORT, (Talon) Hardware.Motors.talon(RF_MOTOR_PORT));
    	//LiveWindow.addActuator("Right Rear Motor", RR_MOTOR_PORT, (Talon) Hardware.Motors.talon(RR_MOTOR_PORT));
    	drivetrain = new TankDrive(leftDriveMotor, rightDriveMotor);
    	
    	//Setup manipulators
    	ballSuckMotor = Motor.compose(Hardware.Motors.talon(LEFT_BALL_SUCK),
    									Hardware.Motors.talon(RIGHT_BALL_SUCK).invert());
    	//LiveWindow.addActuator("Left Ball Suck", LEFT_BALL_SUCK, (Talon) Hardware.Motors.talon(LEFT_BALL_SUCK));
     	//LiveWindow.addActuator("Right Ball Suck", RIGHT_BALL_SUCK, (Talon) Hardware.Motors.talon(RIGHT_BALL_SUCK));
    	ballShootMotor = Motor.compose(Hardware.Motors.talon(LEFT_BALL_SHOOT),
    									Hardware.Motors.talon(RIGHT_BALL_SHOOT).invert());
    	//LiveWindow.addActuator("Left Ball Shoot", LEFT_BALL_SHOOT, (Talon) Hardware.Motors.talon(LEFT_BALL_SHOOT));
     	//LiveWindow.addActuator("Right Ball Shoot", RIGHT_BALL_SHOOT, (Talon) Hardware.Motors.talon(RIGHT_BALL_SHOOT));

     	//Setup joysticks
    	leftDriveStick = Hardware.HumanInterfaceDevices.logitechAttack3D(LEFT_DRIVESTICK_PORT);
    	rightDriveStick = Hardware.HumanInterfaceDevices.logitechAttack3D(RIGHT_DRIVESTICK_PORT);
    	manipulatorStick = Hardware.HumanInterfaceDevices.xbox360(MANIPULATOR_STICK_PORT);
    	
    	//Setup sensors
    	accel = Hardware.Accelerometers.accelerometer(ACCEL_PORT, ACCEL_RANGE);
    	//LiveWindow.addSensor("Accelerometer", 1, accel); //I have no idea how to add the Strongback Accelerometer to a LiveWindow
    	gyro = Hardware.AngleSensors.gyroscope(GYRO_PORT);
    	//LiveWindow.addSensor("Gyroscope", 0, gyro); //I have no idea how to add the Strongback Gyroscope to a LiveWindow
    	leftEncoder = Hardware.AngleSensors.encoder(LEFT_ENCOODER_PORT_A, LEFT_ENCOODER_PORT_B, ENCOODER_PULSE_DISTANCE);
    	leftEncoderAdjusted = AngleSensor.invert(leftEncoder);
    	rightEncoder = Hardware.AngleSensors.encoder(RIGHT_ENCOODER_PORT_A, RIGHT_ENCOODER_PORT_B, ENCOODER_PULSE_DISTANCE);
    	//LiveWindow.addSensor("Left Encoder", LEFT_ENCOODER_PORT_A, leftEncoder);
    	//LiveWindow.addSensor("Right Encoder", RIGHT_ENCOODER_PORT_A, rightEncoder);
    	
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
    	
    	Strongback.configure().recordNoEvents().recordNoData().useExecutionPeriod(50, TimeUnit.MILLISECONDS).initialize();
    }

    public void autonomousInit() {
        // Start Strongback functions ...
        Strongback.start();
    }
    
    public void autonomousPeriodic() {
    	switch(AUTON_MODE) {
    	case 0: Strongback.submit(new DoNothing());
    		break;
    	case 1:
    		if (Math.abs(leftEncoderAdjusted.getAngle()) < AUTON_DISTANCE &&
    				Math.abs(rightEncoder.getAngle()) < AUTON_DISTANCE) {
    					Strongback.submit(new DriveForward(drivetrain, AUTON_SPEED));
    		}
    		break;
    	case 2:
    		if (Math.abs(leftEncoderAdjusted.getAngle()) < AUTON_DISTANCE &&
    				Math.abs(rightEncoder.getAngle()) < AUTON_DISTANCE) {
        				Strongback.submit(new DriveBackward(drivetrain, AUTON_SPEED));
        			}
    		break;
    	default:
    		Strongback.submit(new DoNothing());
    		break;
    	}
    	
    	/*System.out.println("Gyro Angle " + gyro.getAngle());
    	System.out.println();
    	System.out.println("X-Axis " + accel.getXDirection().getAcceleration());
    	System.out.println("Y-Axis " + accel.getYDirection().getAcceleration());
    	System.out.println("Z-Axis " + accel.getZDirection().getAcceleration());*/
    	System.out.println();
    	System.out.println("Left Distance " + leftEncoderAdjusted.getAngle());
    	System.out.println("Right Distance " + rightEncoder.getAngle());
    	System.out.println();
    	System.out.println();
    }
    
    public void teleopInit() {
        // Restart Strongback functions ...
        Strongback.restart();
    }

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
    	System.out.println("Left Distance " + leftEncoderAdjusted.getAngle());
    	System.out.println("Right Distance " + rightEncoder.getAngle());
    	System.out.println();
    	System.out.println();
    }

    public void disabledInit() {
    	drivetrain.stop();
        // Tell Strongback that the robot is disabled so it can flush and kill commands.
        Strongback.disable();
    }
    
    public void disabledPeriodic() {
    	//This section is used for testing only.
    	/*
    	System.out.println("Axis 0 " + manipulatorStick.getAxis(0).read());
    	System.out.println("Axis 1 " + manipulatorStick.getAxis(1).read());
    	System.out.println("Axis 2 " + manipulatorStick.getAxis(2).read());
    	System.out.println("Axis 3 " + manipulatorStick.getAxis(3).read());
    	System.out.println("Axis 4 " + manipulatorStick.getAxis(4).read());
    	System.out.println("Axis 5 " + manipulatorStick.getAxis(5).read());
    	System.out.println("A " + manipulatorStick.getA().isTriggered());
    	System.out.println("B " + manipulatorStick.getB().isTriggered());
    	System.out.println("X " + manipulatorStick.getX().isTriggered());
    	System.out.println("Y " + manipulatorStick.getY().isTriggered());
    	System.out.println("Left Bumper " + manipulatorStick.getLeftBumper().isTriggered());
    	System.out.println("Right Bumper " + manipulatorStick.getRightBumper().isTriggered());
    	System.out.println("Start " + manipulatorStick.getStart().isTriggered());
    	System.out.println("Select " + manipulatorStick.getSelect().isTriggered());
    	System.out.println("Left Trigger " + manipulatorStick.getLeftTrigger().read());
    	System.out.println("Right Trigger " + manipulatorStick.getRightTrigger().read());
    	*/
    }
}
