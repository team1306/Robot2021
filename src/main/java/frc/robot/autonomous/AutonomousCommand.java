package frc.robot.autonomous;

import java.util.Set;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;
// import edu.wpi.first.wpilibj.command.CommandGroup.addSeqential;

/**
 * This is the shell code from the wpilib article "convert simple auto command
 * auto" AutonomousCommand is a sequence of multiple commands (currently only
 * MoveOffLine) Following the constructor of AutonomousCommand, we have a nested
 * class MoveOffLine which represents the Command called in "addSequential()".
 * Each method within MoveOffLine follows the normal command structure of
 * initialization (once), execute (ran repeatedly), isFinished (??), and end
 * (once) with the addition of inturrupt being called if another Command would
 * need to use the same subsystems as AutoCommand
 */
public class AutonomousCommand extends CommandGroup {
    public AutonomousCommand() {
        addSequential(new MoveOffLine());
    }

    public class MoveOffLine extends Command {

        public MoveOffLine() {
            // TODO stop ignoring it
        }

        // Called just before this Command (MoveOffLine) runs the first time
        protected void initialize() {
            //
        }

        // Called repeatedly when this Command (MoveOffLine) is scheduled to run
        protected void execute() {

        }

        // Make this return true when this Command no longer needs to run execute()
        @Override
        protected boolean isFinished() {
            // run robot until it "senses" carpet
            return false;
        }

        // Called once after isFinished returns true
        protected void end() {
            // stop moving
        }

        // Called when another command which requires one or more of the same
        // subsystems is scheduled to run
        protected void inturrupted() {
            end();
        }

        // @Override
        // public Set<Subsystem> getRequirements() {
        // // should require a drivetrain base , swervedrive, but since nothing else is
        // // using
        // // swervedrive, we don't have requirements
        // return null;
        // }

    }
}
