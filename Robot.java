/**
 * Brought to you by 7591 programmer Gavin Hua
 */

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

public class Robot extends TimedRobot {

	// Change the initialization heree if the control scheme is not a joystick
	private Joystick joys = new Joystick(Constants.joys_ID);

	private Translation2d FL2d = new Translation2d(Constants.Length / 2, Constants.Width / 2);
	private Translation2d FR2d = new Translation2d(Constants.Length / 2, -Constants.Width / 2);
	private Translation2d BL2d = new Translation2d(-Constants.Length / 2, Constants.Width / 2);
	private Translation2d BR2d = new Translation2d(-Constants.Length / 2, -Constants.Width / 2);
	private SwerveDriveKinematics SDK = new SwerveDriveKinematics(FL2d, FR2d, BL2d, BR2d);

	private SwerveModuleState[] states;
	private ChassisSpeeds cSpeeds;

	// Change the initialization here if motors are not TalonFXs or if encoders are not CANcoders
	private CANCoder encoderFL = new CANCoder(Constants.encoders[0]);
	private CANCoder encoderFR = new CANCoder(Constants.encoders[1]);
	private CANCoder encoderBL = new CANCoder(Constants.encoders[2]);
	private CANCoder encoderBR = new CANCoder(Constants.encoders[3]);

	private TalonFX steerFL = new TalonFX(Constants.steerMotors[0]);
	private TalonFX steerFR = new TalonFX(Constants.steerMotors[1]);
	private TalonFX steerBL = new TalonFX(Constants.steerMotors[2]);
	private TalonFX steerBR = new TalonFX(Constants.steerMotors[3]);

	private TalonFX driveFL = new TalonFX(Constants.driveMotors[0]);
	private TalonFX driveFR = new TalonFX(Constants.driveMotors[1]);
	private TalonFX driveBL = new TalonFX(Constants.driveMotors[2]);
	private TalonFX driveBR = new TalonFX(Constants.driveMotors[3]);

	private PIDController pidFL = new PIDController(Constants.kp, Constants.ki, Constants.kd);
	private PIDController pidFR = new PIDController(Constants.kp, Constants.ki, Constants.kd);
	private PIDController pidBL = new PIDController(Constants.kp, Constants.ki, Constants.kd);
	private PIDController pidBR = new PIDController(Constants.kp, Constants.ki, Constants.kd);

	public void motorPID(double angle, CANCoder encoder, TalonFX motor, PIDController pid, double offset) {
		double currentPos = encoder.getPosition() + offset;
		pid.setSetpoint(angle);
		double out = pid.calculate(currentPos);
		motor.set(ControlMode.PercentOutput, out);
	}

	@Override
	public void robotInit() {
		pidFL.setTolerance(Constants.tolerance);
		pidFR.setTolerance(Constants.tolerance);
		pidBL.setTolerance(Constants.tolerance);
		pidBR.setTolerance(Constants.tolerance);
	}

	@Override
	public void robotPeriodic() {
	}

	@Override
	public void autonomousInit() {
	}

	@Override
	public void autonomousPeriodic() {
	}

	@Override
	public void teleopInit() {
	}

	@Override
	public void teleopPeriodic() {
		double joysY = joys.getY();
		double joysX = joys.getX();
		double joysZ = joys.getZ();

		double vX = 0;
		double vY = 0;
		double omega = 0;

		if (Math.abs(joysY) > Constants.xBlind) {vX = joysY;}
		if (Math.abs(joysX) > Constants.yBlind) {vY = joysX * -1;}
		if (Math.abs(joysZ) > Constants.zBlind) {omega = joysZ;}

		cSpeeds = new ChassisSpeeds(vX, vY, omega);

		states = SDK.toSwerveModuleStates(cSpeeds);

		driveFL.set(ControlMode.PercentOutput, states[0].speedMetersPerSecond * Constants.driveMod);
		driveFR.set(ControlMode.PercentOutput, states[1].speedMetersPerSecond * Constants.driveMod);
		driveBL.set(ControlMode.PercentOutput, states[2].speedMetersPerSecond * Constants.driveMod);
		driveBR.set(ControlMode.PercentOutput, states[3].speedMetersPerSecond * Constants.driveMod);

		double angleFL = states[0].angle.getDegrees();
		double angleFR = states[1].angle.getDegrees();
		double angleBL = states[2].angle.getDegrees();
		double angleBR = states[3].angle.getDegrees();

		// Optimization due to encoder offset problems. 
		// Delete the if statement if it is causing problems.
		if (vX == vY && vY == omega && omega ==0){
			angleFL += 180;
			angleFR += 180;
			angleBL += 180;
			angleBR += 180;
		}
		motorPID(angleFL, encoderFL, steerFL, pidFL, Constants.offsetFL);
		motorPID(angleFR, encoderFR, steerFR, pidFR, Constants.offsetFR);
		motorPID(angleBL, encoderBL, steerBL, pidBL, Constants.offsetBL);
		motorPID(angleBR, encoderBR, steerBR, pidBR, Constants.offsetBR);
	}

	@Override
	public void disabledInit() {
	}

	@Override
	public void disabledPeriodic() {
	}

	@Override
	public void testInit() {
	}

	@Override
	public void testPeriodic() {
	}
}
