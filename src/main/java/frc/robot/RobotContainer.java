/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.JoystickFlywheel;
import frc.robot.commands.UserDrive;
import frc.robot.commands.VisionCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.Controller;
import frc.robot.utils.Lights;
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

  private JoystickButton visionToggle;

  // subsystem functionality. Subsystems and commands are not initialized unless
  // flagged as true in this section. Important to distinguish this from actually
  // enabling the robot.
  private final boolean drivetrainEnabled = true;
  private final boolean shooterEnabled = false;
  private final boolean intakeEnabled = false;
  private final boolean climberEnabled = false;
  private final boolean lightsEnabled = true;
  private final boolean visionEnabled = true && drivetrainEnabled;// & shooterEnabled;

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
    if(visionEnabled){
      initVision();
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
    Robot.driveTrain= driveTrain;
    new UserDrive(driveTrain, driveRight, driveLeft);
  }

  private void initShooter() {
    shooter = new Shooter();
    Robot.shooter= shooter;
    new JoystickFlywheel(shooter, flywheelSpeed);
  }

  private void initIntake() {
    intake = new Intake();
    Robot.intake=intake;
    new IntakeCommand(intake, intakeSpeed, isIntakeStuck);
  }

  private void initClimber() {
    climber = new Climber();
  }

  private void initLights() {
    lights = new Lights();
  }

  private void initVision(){
    VisionCommand visionCommand = new VisionCommand(driveTrain, shooter);
    visionToggle=Controller.getJoystickButton(Controller.PRIMARY,Controller.BUTTON_X);
    visionToggle.toggleWhenPressed(visionCommand);
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
