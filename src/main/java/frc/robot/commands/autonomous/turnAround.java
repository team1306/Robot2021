package frc.robot.commands.autonomous;

import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.wpilibj.command.Command;
//import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An example command that uses an example subsystem.
 */
public class TurnAround extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final SwerveDrive m_subsystem = new SwerveDrive(); 

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public TurnAround() {
        
    }

    /**
     * Called once the command is initialized. Resets and starts the timer
     */
    @Override
    public void initialize() {
        m_subsystem.resetEncoders();
    }

    /**
     * Called after initialization. Repeatedly drives forward?
     */
    @Override
    public void execute() {
        m_subsystem.swerveDrive(0,0,0.3 - m_subsystem.getFrontRightEnc() * 0.01);
    }

    /**
     * Called whlie execute() is running. Checks if an amount of time has passed
     */
    @Override
    public boolean isFinished() {
        return m_subsystem.getFrontRightEnc() >= 180;
    }

    /**
     * Called once the time has passed to stop the swerveDrive
     */
    public void end() {
        m_subsystem.swerveDrive(0, 0, 0);
    }

    /**
     * Called if something screws up
     */
    public void inturrupted() {
        end();
    }
}