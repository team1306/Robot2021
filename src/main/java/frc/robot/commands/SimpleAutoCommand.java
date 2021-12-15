package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

public class SimpleAutoCommand extends CommandBase {
    public final SwerveDrive m_swerveDrive;

    public SimpleAutoCommand(SwerveDrive swerveDrive) {
        m_swerveDrive = swerveDrive;
    }

    /**
     * Called when the command is initially scheduled.
     */
    @Override
    public void initialize() {}

    /**
     * Called every time the scheduler runs while the command is scheduled.
     */
    @Override
    public void execute() {
        m_swerveDrive.driveTrain(5, 0);
    }
}