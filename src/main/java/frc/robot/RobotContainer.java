/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

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

  // subsystem functionality. Subsystems and commands are not initialized unless
  // flagged as true in this section. Important to distinguish this from actually
  // enabling the robot.
  private final boolean drivetrainEnabled = false;
  private final boolean shooterEnabled = true;
  private final boolean intakeEnabled = false;
  private final boolean climberEnabled = false;
  private final boolean lightsEnabled = true;

  /**
   * Initialization for the robot. Initializies the user inputs, subsystems, and
   * commands, and sets the auto and test comamnds.
   */
  public RobotContainer() {

    configureButtonBindings();

    // TODO
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
    if (lightsEnabled) {
      initLights();
    }
  }

  /**
   * Uses the system of UserAnalog and UserDigital to create the inputs requested
   * by the commands. These inputs are set to the instance variables, and pulled
   * later when the subsystem is initialized.
   */
  private void configureButtonBindings() {
    //Sets up the controller objects so buttons can start to be created
    Controller.init();

    //Drivetrain
    driveRight = Controller.simpleAxis(Controller.PRIMARY, Controller.AXIS_RY);
    driveLeft = Controller.simpleAxis(Controller.PRIMARY, Controller.AXIS_LY);

    //Shooter (will eventually be controlled by Vision)
    flywheelSpeed = Controller.simpleAxis(Controller.PRIMARY, Controller.AXIS_LY);

    //Intake
    intakeSpeed = UserAnalog.fromDigital(Controller.simpleButton(Controller.PRIMARY, Controller.BUTTON_RBUMPER), 1, 0);
    isIntakeStuck = Controller.simpleButton(Controller.PRIMARY, Controller.BUTTON_LBUMPER);//user button if two balls are stuck
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

  private void initLights() {
    lights = new Lights();
  }

  /**
   * Use this to pass the autonomous command to the Robot class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return autoCommand;
  }
}
