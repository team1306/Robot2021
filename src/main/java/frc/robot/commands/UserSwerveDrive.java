/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/
package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;

import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utils.UserAnalog;
import frc.robot.utils.UserDigital;

/**
 * The UserSwerveDrive command uses the SwerveDrive command to execute actions and gather data
 */
public class UserSwerveDrive extends CommandBase {
	public final SwerveDrive m_swerveDrive;
	private final UserAnalog driveX;
	private final UserAnalog driveY;
	// private UserAnalog turn;
	private final UserAnalog turnLeft;
	private final UserAnalog turnRight;
	// private final UserDigital reset;

	private final UserDigital X, Y, A, B;

	/**
	 * Default constructor for UserSwerveDrive. Passes a SwerveDrive object and assigns the
	 * UserAnalog controls to each variable.
	 * 
	 * @param m_swerveDrive
	 * @param driveX        analog input for left/right movement
	 * @param driveY        analog input for up/down movement
	 * @param turnRight     analog input for turning right
	 * @param turnLeft      analog input for turning left
	 * @param x             digital input for X button
	 * @param y             digital input for Y button
	 * @param a             digital input for A button
	 * @param b             digital input for B button
	 */
	public UserSwerveDrive(
		SwerveDrive m_swerveDrive,
		UserAnalog driveX,
		UserAnalog driveY,
		UserAnalog turnRight,
		UserAnalog turnLeft,
		UserDigital x,
		UserDigital y,
		UserDigital a,
		UserDigital b
	) {
		this.m_swerveDrive = m_swerveDrive;
		this.addRequirements(m_swerveDrive);
		this.turnRight = turnRight;
		this.turnLeft = turnLeft;
		// turn = turnRight;
		// turn = () -> turnLeft.get()-turnRight.get();s
		X = x;
		Y = y;
		A = a;
		B = b;

		this.driveX = driveX;
		this.driveY = driveY;
		this.m_swerveDrive.setDefaultCommand(this);
	}

	/**
	 * Called when the command is initially scheduled.
	 */
	@Override
	public void initialize() {}

	// private boolean frp = false, flp = false, brp = false, blp = false;

	/**
	 * Called every time the scheduler runs while the command is scheduled.
	 */
	@Override
	public void execute() {
		double turnTarget = deadzone(turnLeft.get() - turnRight.get());

		// rotate <X,Y> by yaw to achieve field oriented drive
		// https://en.wikipedia.org/wiki/Rotation_of_axes#Derivation
		double x = deadzone(driveX.get());
		double y = deadzone(driveY.get());
		double theta = SwerveDrive.getYaw() * Math.PI / 180;
		double h = Math.cos(theta), v = Math.sin(theta);

		double driveXTarget = x * h + y * v;
		double driveYTarget = y * h - x * v;



		// create vector reprersents (drivex, drivey) in polar coors; rotate theta by
		// yaw degrees,
		// then convert back to cartesian coords
		m_swerveDrive.driveTrain(driveXTarget, -driveYTarget, -turnTarget);
		m_swerveDrive.resetGyro(A.get());

		smartdashboard();
	}

	/**
	 * 
	 * @param input the joystick value that should have a deadzone
	 * @return input if the absoluteValue of input is greater than .1 otherwise 0
	 */
	public double deadzone(double input) {
		if (Math.abs(input) > .05)
			return input * Math.abs(input);

		return 0;
	}

	/**
	 * Called once the command ends or is inturrupted.
	 */
	@Override
	public void end(boolean interrupted) {
		// m_swerveDrive.driveTrain(0, 0, 0);
	}

	/**
	 * Returns true when the command should end.
	 */
	@Override
	public boolean isFinished() {
		return false;
	}

	public void setModuleStates(SwerveModuleState[] states) {
		m_swerveDrive.setModuleStates(states);
	}

	public void smartdashboard() {
		m_swerveDrive.frontLeft.shuffleboard("Front Left");
		m_swerveDrive.backLeft.shuffleboard("Back Left");
		m_swerveDrive.frontRight.shuffleboard("Front Right");
		m_swerveDrive.backRight.shuffleboard("Back Right");
		SmartDashboard.putNumber("X Joystick Value", driveX.get());
		SmartDashboard.putNumber("Y Joystick Value", driveY.get());
		SmartDashboard.putNumber(
			"Turn Value",
			turnLeft.get() - turnRight.get()
		);
	}
}