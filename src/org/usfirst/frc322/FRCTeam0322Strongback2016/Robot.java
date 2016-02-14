/* Created Sat Jan 23 12:56:39 EST 2016 */
package org.usfirst.frc322.FRCTeam0322Strongback2016;

import org.strongback.Strongback;
import org.strongback.components.Motor;
import org.strongback.components.ui.ContinuousRange;
import org.strongback.components.ui.FlightStick;
import org.strongback.components.ThreeAxisAccelerometer;
import org.strongback.components.AngleSensor;
import org.strongback.drive.TankDrive;
import org.strongback.hardware.Hardware;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.interfaces.Accelerometer.Range;

public class Robot extends IterativeRobot {

	private static final int LEFT_DRIVESTICK_PORT = 0;
	private static final int RIGHT_DRIVESTICK_PORT = 1;
	private static final int LF_MOTOR_PORT = 0;
	private static final int LR_MOTOR_PORT = 1;
	private static final int RF_MOTOR_PORT = 3;
	private static final int RR_MOTOR_PORT = 2;
	private static final SPI.Port ACCEL_PORT = SPI.Port.kOnboardCS1;
	private static final Range ACCEL_RANGE = Range.k2G;
	private static final SPI.Port GYRO_PORT = SPI.Port.kOnboardCS0;

	private TankDrive drive;
	private ContinuousRange leftSpeed;
	private ContinuousRange rightSpeed;

	private ThreeAxisAccelerometer accel;
	private AngleSensor gyro;
	
	public static CameraServer cameraServer;
	
    @Override
    public void robotInit() {
    	Motor left = Motor.compose(Hardware.Motors.talon(LF_MOTOR_PORT),
    								Hardware.Motors.talon(LR_MOTOR_PORT)).invert();
    	Motor right = Motor.compose(Hardware.Motors.talon(RF_MOTOR_PORT),
									Hardware.Motors.talon(RR_MOTOR_PORT));
    	drive = new TankDrive(left, right);
    	
    	FlightStick leftDriveStick = Hardware.HumanInterfaceDevices.logitechAttack3D(LEFT_DRIVESTICK_PORT);
    	FlightStick rightDriveStick = Hardware.HumanInterfaceDevices.logitechAttack3D(RIGHT_DRIVESTICK_PORT);
    	leftSpeed = leftDriveStick.getPitch();
    	rightSpeed = rightDriveStick.getPitch();
    	
    	accel = Hardware.Accelerometers.accelerometer(ACCEL_PORT, ACCEL_RANGE);
    	gyro = Hardware.AngleSensors.gyroscope(GYRO_PORT);
    	
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

}
