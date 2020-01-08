# Robot2020 
Team 1306 Repository for the 2020 First RISE game.

# Conventions
For the purposes of the code and documentation:
*   "Front" shall be defined as the side of the robot with the shooter
*   "Left" shall be the side of the robot on the left-hand side when the same direction as front
*   "Right" shall be the other side

## File Purposes
*   `Main.java` - Entry point. Little change ever needed.
*   `Constants.java` - As the name implies, stores the constants we need for the robot. These are usually the port numbers for connecting to the rio or the CAN id's, and if we need to define color values or something similar it would go here. PID or encoder values assosciated with a specific subsystem, however, go to that subsystem.
*   `Robot.java` - With the new season, this folder is in flux. Most robot setup is done in RobotContainer.java, so this file might be used for Phase-specific triggers and periodic calls.
*   `RobotContainer.java` - Setup of the robot and any needed initialization

#### Subsystems 
Located in /subsystems
*   `Drivetrain.java`
*   `Intake.java` - Controls the intake AND the handoff to the indexer
*   `Shooter.java` - Indexer and shooter, NOT including aiming- that's from the command
*   `Spinner.java` - Mechanism and sensors for manipulating the control panel
*   `Climber.java`

#### Commands
Located in /commands
*   `UserDrive.java` - Drive with user input

#### Utils
Located in /utils
Files meant to ease the implementation of a variety of other files
*   `UserInput.java` - Contains the [functional interfaces](https://www.geeksforgeeks.org/functional-interfaces-java/) for passing llamdas to commands to grab user inputs
    *   `UserAnalog` - returns a -1 to 1 value. These bounds are not checked: be sure the bounds are enforced for reliable results.
    *   `UserDigital` - returns a boolean
*   `PIDTunerCommand` - A command to harness the ShuffleBoard for tuning PID gains for different subsystems throughout the season.
*   `PIDSetup.java` - A tool for easing the implementation of PID loops by controlling verbosity and fixing errors in one spot.
*   `Encoder.java` - A location for managing the unit transformations for different types of encoders.
*   `Controller.java` - A list of all the XBox Controller mappings, as well as access methods for creating simple user inputs