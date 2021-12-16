package frc.robot.commands;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    public void initialize() {
        m_swerveDrive.resetEncoders();

    }

    /**
     * Called every time the scheduler runs while the command is scheduled.
     */
    @Override
    public void execute() {
        if (m_swerveDrive.getDegress() < 20000) {
            m_swerveDrive.driveTrain(1, 0, 0);
        } else {
            m_swerveDrive.driveTrain(0, 0, 0);
        }
        SmartDashboard.putNumber("Rotation", m_swerveDrive.getDegress());
    }
}