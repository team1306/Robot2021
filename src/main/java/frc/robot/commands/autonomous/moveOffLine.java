package frc.robot.commands.autonomous;

import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
//import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Command that moves the robot off of the starting line.
 */
public class MoveOffLine extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final SwerveDrive m_subsystem = new SwerveDrive(); 

    private final double time;
    private final Timer timer;

    /**
     * Creates MoveOffLine command.
     * Declares a timer and sets the SwerveDrive subsystem to move forward
     */
    public MoveOffLine(double time) {
        timer = new Timer();
        this.time = time;
        m_subsystem.swerveDrive(0, 1, 0);
    }

    /**
     * Called once the command is initialized. Resets and starts the timer
     */
    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    /**
     * Called after initialization. Repeatedly drives forward?
     */
    @Override
    public void execute() { // call MoveOffLine?
        m_subsystem.swerveDrive(0, 1, 0);
    }

    /**
     * Called whlie execute() is running. Checks if an amount of time has passed
     */
    @Override
    public boolean isFinished() {
        return timer.hasElapsed(time);
    }

    /**
     * Called once the time has passed to stop the swerveDrive
     */
    public void end() {
        m_subsystem.swerveDrive(0, 0, 0);
        //m_subsystem.turnWheelsIn() write this method?
    }

    /**
     * Called if something screws up
     */
    public void inturrupted() {
        end();
    }
}