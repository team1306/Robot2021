# Robot2020 
Team 1306 Repository for the 2020 First RISE game.

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
    *   `Analog` - returns a -1 to 1 value
    *   `Digital` - returns a boolean
