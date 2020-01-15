/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.JoystickFlywheel;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.Controller;
import frc.robot.utils.Encoder;
import frc.robot.utils.PIDTunerCommand;
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

  private final DriveTrain driveTrain;
  public final Shooter shooter;
  public final Intake intake;  

  // user controlls
  private UserAnalog driveRight;
  private UserAnalog driveLeft;
  private UserAnalog flywheelSpeed;
  private UserAnalog intakeSpeed;
  // inputs

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    // driveTrain = new DriveTrain();
    shooter = new Shooter();
    driveTrain = null;
    intake = new Intake();
    configureButtonBindings();
    configureCommands();
    autoCommand = null;
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    Controller.init();
    // UserAnalog driveRight = Controller.simpleAxis(Controller.PRIMARY,
    // Controller.AXIS_RY);
    // UserAnalog driveLeft = Controller.simpleAxis(Controller.PRIMARY,
    // Controller.AXIS_LY);
    flywheelSpeed = Controller.simpleAxis(Controller.PRIMARY, Controller.AXIS_LY);
    intakeSpeed = UserAnalog.fromDigital(Controller.simpleButton(Controller.PRIMARY, Controller.BUTTON_RBUMPER),1,0);
  }

  private void configureCommands() {
    // drive

    // shooter
    JoystickFlywheel testShooter = new JoystickFlywheel(shooter, flywheelSpeed);
    PIDTunerCommand tuneShooter = new PIDTunerCommand(ControlMode.Velocity, -1, 1, false, FeedbackDevice.QuadEncoder,
        new SubsystemBase[] { shooter }, Encoder.VersaPlanetary, shooter.flywheel);
    Robot.testCommand = tuneShooter;

    //intake
    IntakeCommand intakeCommand = new IntakeCommand(intake, intakeSpeed);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return autoCommand;
  }
}
