package frc.robot.commands;

import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utils.UserAnalog;

/**
 * An example command that uses an example subsystem.
 */
public class IntakeCommand extends CommandBase {
    private final UserAnalog speed;
    private final Intake intake;

    /**
     * Creates a new IntakeCommand.
     */
    public IntakeCommand(UserAnalog speed, Intake intake) {
        this.speed = speed;
        this.intake = intake;
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        intake.spin(speed.get());

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        intake.spin(0.0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}