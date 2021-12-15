package frc.robot.autonomous;

import java.util.Set;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;
// import edu.wpi.first.wpilibj.command.CommandGroup.addSeqential;

/**
 * This is the shell code from the wpilib article "command based programming" AutonomousCommand is a
 * sequence of multiple commands (currently only MoveOffLine) Following the constructor of
 * AutonomousCommand, we have a nested class MoveOffLine which represents the Command called in
 * "addCommands()". Each method within MoveOffLine follows the normal command structure of
 * initialization (once), execute (ran repeatedly), isFinished (??), and end (once)
 */
public class AutonomousCommand extends SequentialCommandGroup {
    public AutonomousCommand() {
        addCommands(new MoveOffLine());
    }

    public class MoveOffLine extends CommandBase {

        public MoveOffLine() {
            // TODO stop ignoring it
        }

        // Called just before this Command (MoveOffLine) runs the first time
        public void initialize() {
            //
        }

        // Called repeatedly when this Command (MoveOffLine) is scheduled to run
        public void execute() {

        }

        // Make this return true when this Command no longer needs to run execute()
        @Override
        public boolean isFinished() {
            // run robot until it "senses" carpet
            return false;
        }

        // Called once after isFinished returns true
        public void end() {
            // stop moving
        }
    }
}
