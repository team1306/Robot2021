/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ClimberCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.JoystickFlywheel;
import frc.robot.commands.SpinnerCommand;
import frc.robot.commands.SpinnerEncoderCommand;
import frc.robot.commands.UserDrive;
import frc.robot.commands.VisionCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spinner;
import frc.robot.utils.Controller;
import frc.robot.utils.Lights;
import frc.robot.utils.TestVisionCommand;
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

  // user controls
  private UserAnalog driveRight;   //drive
  private UserAnalog driveLeft;
  private JoystickButton shiftHigh;
  private JoystickButton shiftLow;

  private UserAnalog flywheelSpeed;//shooter
  private JoystickButton hoodToggleButton;

  private UserAnalog intakeSpeed;  //intake
  private UserDigital isIntakeStuck;
  private UserDigital indexOverride;
  private JoystickButton intakeUp;
  private JoystickButton intakeDown;

  private UserDigital climbButton; //climber
  private UserDigital climbReverse;

  private UserDigital spinButton;  //spinner (todo)
  private JoystickButton spin1;
  private JoystickButton spin32;


  private JoystickButton visionToggle;
  

  // subsystem functionality. Subsystems and commands are not initialized unless
  // flagged as true in this section. Important to distinguish this from actually
  // enabling the robot.
  private final boolean drivetrainEnabled = true;
  private final boolean shooterEnabled = true;
  private final boolean intakeEnabled = true;
  private final boolean climberEnabled = true;
  private final boolean lightsEnabled = true;
  private final boolean visionEnabled = true && drivetrainEnabled & shooterEnabled;
  private final boolean spinnerEnabled = true;
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
    if(spinnerEnabled){
      initSpinner();
    }

    Robot.testCommand = new TestVisionCommand();
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
    UserAnalog driveForward = Controller.simpleAxis(Controller.PRIMARY, Controller.AXIS_RTRIGGER);
    UserAnalog driveBackward = Controller.simpleAxis(Controller.PRIMARY, Controller.AXIS_LTRIGGER);
    UserAnalog driveRotation = Controller.simpleAxis(Controller.PRIMARY, Controller.AXIS_LX);
    DriveDirection driveDirection = new DriveDirection();//select which direction is forward
    Controller.bindCallback(Controller.PRIMARY, Controller.BUTTON_LBUMPER, driveDirection::swap);
    //arcade drive to tank drive conversion
    UserAnalog[] driveParts = UserDrive.arcadeToTankAdditive(()->{return driveDirection.get()*(driveForward.get()-driveBackward.get());}, driveRotation);
    driveRight=driveParts[0];
    driveLeft=driveParts[1];
    
    shiftHigh = Controller.getJoystickButton(Controller.PRIMARY, Controller.BUTTON_START);
    shiftLow = Controller.getJoystickButton(Controller.PRIMARY, Controller.BUTTON_BACK);

    visionToggle=Controller.getJoystickButton(Controller.SECONDARY,Controller.BUTTON_Y);

    //Shooter (will eventually be controlled by Vision)
    flywheelSpeed = Controller.simpleAxis(Controller.SECONDARY, Controller.AXIS_LTRIGGER);
    hoodToggleButton = Controller.getJoystickButton(Controller.PRIMARY, Controller.BUTTON_RBUMPER);

    //Intake
    intakeSpeed = Controller.simpleAxis(Controller.SECONDARY, Controller.AXIS_RTRIGGER);
    isIntakeStuck = Controller.simpleButton(Controller.SECONDARY, Controller.BUTTON_LBUMPER);//user button if two balls are stuck
    indexOverride = Controller.simpleButton(Controller.SECONDARY, Controller.BUTTON_RBUMPER);
    intakeUp = Controller.getJoystickButton(Controller.PRIMARY, Controller.BUTTON_Y);
    intakeDown = Controller.getJoystickButton(Controller.PRIMARY, Controller.BUTTON_B);

    //Climber
    climbButton = Controller.simpleButton(Controller.PRIMARY, Controller.BUTTON_X);
    climbReverse = Controller.simpleButton(Controller.PRIMARY, Controller.BUTTON_A);

    //Spinner
    spinButton = Controller.simpleButton(Controller.SECONDARY, Controller.BUTTON_A);
    spin1 = Controller.getJoystickButton(Controller.SECONDARY, Controller.BUTTON_X);
    spin32 = Controller.getJoystickButton(Controller.SECONDARY, Controller.BUTTON_B);
  }

  private void initDrivetrain() {
    DriveTrain driveTrain = new DriveTrain();
    Robot.driveTrain= driveTrain;
    new UserDrive(driveTrain, driveRight, driveLeft); //is set as default so don't need to track
    shiftHigh.whenPressed(()->{
        driveTrain.shift(DriveTrain.K_HIGH_GEAR);
    });//intentionally not requiring drivetrain to shift
    shiftLow.whenPressed(()->{
      driveTrain.shift(DriveTrain.K_LOW_GEAR);
    });
  }

  private void initShooter() {
    Shooter shooter = new Shooter();
    Robot.shooter= shooter;
    new JoystickFlywheel(shooter, flywheelSpeed);
    hoodToggleButton.whenPressed(()->{shooter.setHood(!shooter.isHoodUp());});
  }

  private void initIntake() {
    Intake intake = new Intake();
    Robot.intake=intake;
    new IntakeCommand(intake, intakeSpeed, isIntakeStuck, indexOverride);
    intakeUp.whenPressed(intake::retract, intake);
    intakeDown.whenPressed(intake::extend, intake);
  }

  private void initClimber() {
    Robot.climber = new Climber();
    new ClimberCommand(Robot.climber, climbButton, climbReverse);
  }

  private void initLights() {
    Robot.lights = new Lights();
  }

  private void initVision(){
    VisionCommand visionCommand = new VisionCommand(Robot.driveTrain, Robot.shooter, Robot.intake);
    visionToggle.toggleWhenPressed(visionCommand);
  }

  private void initSpinner(){
    Robot.spinner = new Spinner();
    new SpinnerCommand(Robot.spinner, spinButton);
    spin1.whenPressed(new SpinnerEncoderCommand(1,Robot.spinner));
    spin32.whenPressed(new SpinnerEncoderCommand(32,Robot.spinner));
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


class DriveDirection implements UserAnalog{

  private double direction = 1;

  public double get(){
    return direction;
  }

  public void swap(){
    direction = -direction;
  }
}