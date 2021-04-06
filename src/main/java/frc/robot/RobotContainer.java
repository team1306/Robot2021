/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
//import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.utils.Controller;
import frc.robot.utils.UserAnalog;
import frc.robot.utils.UserDigital;
import frc.robot.subsystems.Intake;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final Command autoCommand;

    // The robot's inputs that it recieves from the controller are defined here
    private UserAnalog driveX;
    private UserAnalog driveY;
    private UserAnalog turnLeft;
    private UserAnalog turnRight;

    private UserDigital forwardIntake;
    private UserDigital backwardIntake;

    private DriveCommand userDrive;
    private IntakeCommand intakeCommand;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        System.out.println("robotContainer is running");
        Controller.init();

        // configure the button bindings
        configureButtonBindings();

        autoCommand = null;

        // driveX = new UserAnalog();
        System.out.println("Robot Container is running.");

        DriveTrain tankDrive = new DriveTrain();
        userDrive = new DriveCommand(tankDrive, driveX, turnRight, turnLeft);

        Intake m_intake = new Intake();
        intakeCommand = new IntakeCommand(m_intake, forwardIntake, backwardIntake);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        //TODO robot moves backwards 
        driveX = Controller.simpleAxis(Controller.PRIMARY, Controller.AXIS_LY);
        driveY = Controller.simpleAxis(Controller.PRIMARY, Controller.AXIS_RY);

        turnRight = Controller.simpleAxis(Controller.PRIMARY, Controller.AXIS_RTRIGGER);
        turnLeft = Controller.simpleAxis(Controller.PRIMARY, Controller.AXIS_LTRIGGER);
        forwardIntake = Controller.simpleButton(Controller.PRIMARY, Controller.BUTTON_RBUMPER);
        backwardIntake = Controller.simpleButton(Controller.PRIMARY, Controller.BUTTON_LBUMPER);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return null;
    }
}
