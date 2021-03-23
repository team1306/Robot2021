/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.commands.UserSwerveDrive;

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
import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.utils.Controller;
import frc.robot.utils.UserAnalog;

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
    private UserAnalog turn;

    private UserSwerveDrive userSwerveDrive;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        System.out.println("robotContainer is running");
        Controller.init();

        // configure the button bindings
        configureButtonBindings();

        autoCommand = null;

        SwerveDrive driveTrain = new SwerveDrive();
        Robot.swerveDrive = driveTrain;
        userSwerveDrive = new UserSwerveDrive(driveTrain, driveY, driveX, turn);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        driveX = Controller.simpleAxis(Controller.PRIMARY, Controller.AXIS_RX);
        driveY = Controller.simpleAxis(Controller.PRIMARY, Controller.AXIS_RY);
        turn = Controller.simpleAxis(Controller.PRIMARY, Controller.AXIS_LX);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // Create a speed constraint to ensure we don't accelerate too fast
        SwerveDriveKinematicsConstraint autoConstraint  = new SwerveDriveKinematicsConstraint(userSwerveDrive.m_swerveDrive.kinematics, Constants.FASTEST_SPEED_METERS);
        
        // Create config for trajectory
        TrajectoryConfig config =
        new TrajectoryConfig(Constants.FASTEST_SPEED_METERS, 
            Constants.FASTEST_ACCELERATION)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(userSwerveDrive.m_swerveDrive.kinematics)  
        // Apply the voltage constraint
        .addConstraint(autoConstraint);

        String trajectoryJSON = "../deploy/path/Barrel.wpilib.json";
        Trajectory trajectory = new Trajectory(null); // todo what is trajectory list?????
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            System.out.println("Unable to open trajectory");
        }

        TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
            Constants.kMaxAngularSpeedRadians, Constants.kMaxAngularVelocityRadians);
  
        var thetaController =
        new ProfiledPIDController(
            Constants.THETA_CONTROLLER_P, Constants.THETA_CONTROLLER_I, Constants.THETA_CONTROLLER_D, kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // everything below had m_robotDrive replaced with userSwerveDrive

        // TODO add new methods + test
        SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            trajectory,
            userSwerveDrive.m_swerveDrive::getPose(), // Functional interface to feed supplier
            userSwerveDrive.m_swerveDrive.kinematics,
            // Position controllers
            new PIDController(Constants.X_CONTROLLER_P, Constants.X_CONTROLLER_I, Constants.X_CONTROLLER_D),
            new PIDController(Constants.Y_CONTROLLER_P, Constants.Y_CONTROLLER_I, Constants.Y_CONTROLLER_D),
            thetaController,
            userSwerveDrive.m_swerveDrive::setModuleStates,
            userSwerveDrive.m_swerveDrive);

        // Reset odometry to the starting pose of the trajectory.
        userSwerveDrive.m_swerveDrive.resetOdometry(trajectory.getInitialPose());

        // Run path following command, then stop at the end.
        return swerveControllerCommand.andThen(() -> userSwerveDrive.m_swerveDrive.driveTrain(0, 0, 0));
        // changed from .tankDriveVolts(0, 0));
    }
}
