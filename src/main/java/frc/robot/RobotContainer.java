/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/
package frc.robot;

import java.util.ResourceBundle.Control;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.SimpleAutoCommand;
import frc.robot.commands.UserSwerveDrive;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.utils.Controller;
import frc.robot.utils.UserAnalog;
import frc.robot.utils.UserDigital;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

	// The robot's subsystems and commands are defined here...

	// The robot's inputs that it recieves from the controller are defined here
	private UserAnalog driveX;
	private UserAnalog driveY;
	private UserAnalog turnRight;
	private UserAnalog turnLeft;
	//private UserDigital reset;

	private UserDigital forwardIntake;
	private UserDigital backwardIntake;
	//lazy

	private UserDigital x, y, a, b;
	public UserSwerveDrive userSwerveDrive;

	private final SimpleAutoCommand autoCommand;

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		Controller.init();
		configureButtonBindings();


		SwerveDrive driveTrain = new SwerveDrive();
		Robot.swerveDrive = driveTrain;
		userSwerveDrive = new UserSwerveDrive(
			driveTrain, driveX, driveY, turnRight, turnLeft, x, y, a, b
		);
		autoCommand = new SimpleAutoCommand(driveTrain);
	}

	/**
	 * Use this method to define your button->command mappings. Buttons are assigned through the
	 * Controller class. Use simpleAxis if the inputs are analog. Use simpleButton if the inputs are
	 * digital.
	 */
	private void configureButtonBindings() {
		driveX = Controller.simpleAxis(Controller.PRIMARY, Controller.AXIS_LX);
		driveY = Controller.simpleAxis(Controller.PRIMARY, Controller.AXIS_LY);
		turnRight = Controller.simpleAxis(
			Controller.PRIMARY,
			Controller.AXIS_RTRIGGER
		);
		turnLeft = Controller.simpleAxis(
			Controller.PRIMARY,
			Controller.AXIS_LTRIGGER
		);
		//reset = Controller.simpleButton(Controller.PRIMARY, Controller.BUTTON_A);
		forwardIntake = Controller.simpleButton(
			Controller.PRIMARY,
			Controller.BUTTON_RBUMPER
		);
		backwardIntake = Controller.simpleButton(
			Controller.PRIMARY,
			Controller.BUTTON_LBUMPER
		);


		x = Controller.simpleButton(Controller.PRIMARY, Controller.BUTTON_X);
		y = Controller.simpleButton(Controller.PRIMARY, Controller.BUTTON_Y);
		a = Controller.simpleButton(Controller.PRIMARY, Controller.BUTTON_A);
		b = Controller.simpleButton(Controller.PRIMARY, Controller.BUTTON_B);
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		return autoCommand;
	}

}