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
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.JoystickFlywheel;
import frc.robot.commands.UserDrive;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.Controller;
import frc.robot.utils.Lights;
import frc.robot.utils.PIDTunerCommand;
import frc.robot.utils.UserAnalog;
import frc.robot.utils.UserDigital;

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

  private DriveTrain driveTrain;
  private Shooter shooter;
  private Intake intake;
  private Climber climber;

  private Lights lights;

  // user controls
  private UserAnalog driveRight;
  private UserAnalog driveLeft;

  private UserAnalog flywheelSpeed;

  private UserAnalog intakeSpeed;
  private UserDigital isIntakeStuck;

  // subsystem functionality
  private final boolean drivetrainEnabled = false;
  private final boolean shooterEnabled = false;
  private final boolean intakeEnabled = false;
  private final boolean climberEnabled = false;
  private final boolean lightsEnabled = true;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    configureButtonBindings();

    autoCommand = null;

    if (drivetrainEnabled) {
      initDrivetrain();
    }
    if (shooterEnabled) {
      initShooter();
    }
    if (intakeEnabled) {
      initIntake();
    }
    if (climberEnabled) {
      initClimber();
    }
    if (lightsEnabled){
      initLights();
    }
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    Controller.init();
    driveRight = Controller.simpleAxis(Controller.PRIMARY, Controller.AXIS_RY);
    driveLeft = Controller.simpleAxis(Controller.PRIMARY, Controller.AXIS_LY);

    flywheelSpeed = Controller.simpleAxis(Controller.PRIMARY, Controller.AXIS_LY);

    intakeSpeed = UserAnalog.fromDigital(Controller.simpleButton(Controller.PRIMARY, Controller.BUTTON_RBUMPER), 1, 0);
    isIntakeStuck = Controller.simpleButton(Controller.PRIMARY, Controller.BUTTON_LBUMPER);
  }

  private void initDrivetrain() {
    driveTrain = new DriveTrain();
    new UserDrive(driveTrain, driveRight, driveLeft);
  }

  private void initShooter() {
    shooter = new Shooter();
    Robot.testCommand = new PIDTunerCommand(ControlMode.Velocity, 0, 1, false, FeedbackDevice.QuadEncoder, shooter,
        shooter.flywheelEnc, shooter.flywheel);
    new JoystickFlywheel(shooter, flywheelSpeed);
  }

  private void initIntake() {
    intake = new Intake();
    new IntakeCommand(intake, intakeSpeed, isIntakeStuck);
  }

  private void initClimber() {
    climber = new Climber();
  }

  private void initLights(){
    lights = new Lights();
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
