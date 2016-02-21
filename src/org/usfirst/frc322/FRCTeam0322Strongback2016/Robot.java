/* Created Sat Jan 23 12:56:39 EST 2016 */
package org.usfirst.frc322.FRCTeam0322Strongback2016;

import org.strongback.Strongback;
import org.strongback.components.Motor;
import org.strongback.components.ui.ContinuousRange;
import org.strongback.components.ui.FlightStick;
import org.strongback.components.ui.Gamepad;
import org.strongback.components.ThreeAxisAccelerometer;
import org.strongback.components.AngleSensor;
import org.strongback.drive.TankDrive;
import org.strongback.hardware.Hardware;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.interfaces.Accelerometer.Range;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

public class Robot extends IterativeRobot {

	private static final int LEFT_DRIVESTICK_PORT = 0;
	private static final int RIGHT_DRIVESTICK_PORT = 1;
	private static final int MANIPULATOR_STICK_PORT = 2;
	private static final int LF_MOTOR_PORT = 0;
	private static final int LR_MOTOR_PORT = 1;
	private static final int RF_MOTOR_PORT = 3;
	private static final int RR_MOTOR_PORT = 2;
	private static final int LEFT_ENCOODER_PORT_A = 0;
	private static final int LEFT_ENCOODER_PORT_B = 1;
	private static final int RIGHT_ENCOODER_PORT_A = 2;
	private static final int RIGHT_ENCOODER_PORT_B = 3;
	private static final double ENCOODER_PULSE_DISTANCE = 1.0;
	private static final SPI.Port ACCEL_PORT = SPI.Port.kOnboardCS1;
	private static final Range ACCEL_RANGE = Range.k2G;
	private static final SPI.Port GYRO_PORT = SPI.Port.kOnboardCS0;


	private TankDrive drive;
	private ContinuousRange leftSpeed;
	private ContinuousRange rightSpeed;

	private ThreeAxisAccelerometer accel;
	private AngleSensor gyro;
	private AngleSensor leftEncoder;
	private AngleSensor rightEncoder;
	
	public static CameraServer cameraServer;
	
	
    @Override
    public void robotInit() {
    	Motor left = Motor.compose(Hardware.Motors.talon(LF_MOTOR_PORT),
    								Hardware.Motors.talon(LR_MOTOR_PORT)).invert();
    	LiveWindow.addActuator("Left Front Motor", LF_MOTOR_PORT, (Talon) Hardware.Motors.talon(LF_MOTOR_PORT));
    	LiveWindow.addActuator("Left Rear Motor", LR_MOTOR_PORT, (Talon) Hardware.Motors.talon(LR_MOTOR_PORT));
    	Motor right = Motor.compose(Hardware.Motors.talon(RF_MOTOR_PORT),
									Hardware.Motors.talon(RR_MOTOR_PORT));
    	LiveWindow.addActuator("Right Front Motor", RF_MOTOR_PORT, (Talon) Hardware.Motors.talon(RF_MOTOR_PORT));
    	LiveWindow.addActuator("Right Rear Motor", RR_MOTOR_PORT, (Talon) Hardware.Motors.talon(RR_MOTOR_PORT));
    	drive = new TankDrive(left, right);
    	
    	FlightStick leftDriveStick = Hardware.HumanInterfaceDevices.logitechAttack3D(LEFT_DRIVESTICK_PORT);
    	FlightStick rightDriveStick = Hardware.HumanInterfaceDevices.logitechAttack3D(RIGHT_DRIVESTICK_PORT);
    	Gamepad manipulatorStick = xbox360(MANIPULATOR_STICK_PORT);
    	leftSpeed = leftDriveStick.getPitch();
    	rightSpeed = rightDriveStick.getPitch();
    	
    	accel = Hardware.Accelerometers.accelerometer(ACCEL_PORT, ACCEL_RANGE);
    	//LiveWindow.addSensor("Accelerometer", 1, accel);
    	gyro = Hardware.AngleSensors.gyroscope(GYRO_PORT);
    	//LiveWindow.addSensor("Gyroscope", 0, gyro);
    	//leftEncoder = Hardware.AngleSensors.encoder(LEFT_ENCOODER_PORT_A, LEFT_ENCOODER_PORT_B, ENCOODER_PULSE_DISTANCE);
    	//rightEncoder = Hardware.AngleSensors.encoder(RIGHT_ENCOODER_PORT_A, RIGHT_ENCOODER_PORT_B, ENCOODER_PULSE_DISTANCE);    	
    	//LiveWindow.addSensor("Left Encoder", LEFT_ENCOODER_PORT_A, leftEncoder);
    	//LiveWindow.addSensor("Right Encoder", RIGHT_ENCOODER_PORT_A, rightEncoder);
    	
    	cameraServer = CameraServer.getInstance();
        cameraServer.setQuality(25);
        cameraServer.startAutomaticCapture("cam0");
    	
    	Strongback.configure().recordNoEvents().recordNoData().initialize();
    }

    @Override
    public void autonomousInit() {
        // Start Strongback functions ...
        Strongback.start();
    }
    
    @Override
    public void autonomousPeriodic() {
    	
    }
    
    @Override
    public void teleopInit() {
        // Restart Strongback functions ...
        Strongback.restart();
    }

    @Override
    public void teleopPeriodic() {
    	drive.tank(leftSpeed.read(), rightSpeed.read());
    }

    @Override
    public void disabledInit() {
    	drive.stop();
        // Tell Strongback that the robot is disabled so it can flush and kill commands.
        Strongback.disable();
    }
    public static org.strongback.components.ui.Gamepad xbox360(int port) {
    	Joystick joystick = new Joystick(port);
    	return Gamepad.create(joystick::getRawAxis,
    						  joystick::getRawButton,
    						  joystick::getPOV,
    						  () -> joystick.getRawAxis(0),
    						  () -> joystick.getRawAxis(1),
    						  () -> joystick.getRawAxis(3),
    						  () -> joystick.getRawAxis(4),
    						  () -> joystick.getRawAxis(2),
    						  () -> joystick.getRawAxis(2),
    						  () -> joystick.getRawButton(4),
    						  () -> joystick.getRawButton(5),
    						  () -> joystick.getRawButton(0),
    						  () -> joystick.getRawButton(1),
    						  () -> joystick.getRawButton(2),
    						  () -> joystick.getRawButton(3),
    						  () -> joystick.getRawButton(7),
    						  () -> joystick.getRawButton(6),
    						  () -> joystick.getRawButton(8),
    						  () -> joystick.getRawButton(9));
    }
}