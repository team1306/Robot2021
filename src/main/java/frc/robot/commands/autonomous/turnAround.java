package frc.robot.commands.autonomous;

import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.wpilibj.command.Command;

/**
 * TurnAround command that utilizes the SwerveDrive subsystem
 * Turns the robot 180 degrees from its current position
 */
public class TurnAround extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final SwerveDrive m_subsystem = new SwerveDrive(); 

    /**
     * Creates a new TurnAround command
     * TODO this
     */
    public TurnAround() {
        
    }

    /**
     * Called once the command is initialized. Resets encoders to measure new positions
     */
    @Override
    public void initialize() {
        m_subsystem.resetEncoders();
    }

    /**
     * Called after initialization. Starts turning and slows down?
     */
    @Override
    public void execute() {
        m_subsystem.swerveDrive(0,0,0.3 - m_subsystem.getFrontRightEnc() * 0.01);
    }

    /**
     * Called while execute() is running. Checks if the encoder measurement is at 180 degrees
     */
    @Override
    public boolean isFinished() {
        return m_subsystem.getFrontRightEnc() >= 180;
    }

    /**
     * Called once the encoder position is greater than 180
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