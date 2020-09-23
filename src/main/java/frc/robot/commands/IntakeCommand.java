package frc.robot.commands;

import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utils.UserAnalog;
import frc.robot.utils.UserDigital;

/**
 * The Intake command for the Intake subsystem
 */
public class IntakeCommand extends CommandBase {
    private final UserAnalog speed;
    private final UserDigital press;
    private final Intake intake;

    /**
     * Creates a new IntakeCommand.
     */
    public IntakeCommand(UserAnalog speed, Intake intake, UserDigital press) {
        this.speed = speed;
        this.intake = intake;
        this.press = press;

        addRequirements(intake);
        intake.setDefaultCommand(this);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // This will just run both motors at the same speed.
        intake.spin(speed.get()); 

        /*
         * I will want to add a thing to spit out the balls if need be. I don't think we
         * have and sensor stuff to count how many balls are in the index, maybe we do,
         * but I'd have to find it. The method to determine if the balls are jammed
         * should probably go in the Intake subsystem I think.
         */

        /* 
         * I will also have to figure out how to retract the intake with this command.
         * Hopefully with just one button for input
         */
        if (press.get()) 
            intake.extend();
            // TODO what is it doing when not extended? or before the button being pressed
            // TODO Make it toggle with some if else statements to see if it's been extended or retracted 

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // Will just stop the spinning of the intake motors.
        intake.spin(0.0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}