package frc.robot.commands;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

public class SimpleAutoCommand extends CommandBase {
    public final SwerveDrive m_swerveDrive;

    public SimpleAutoCommand(SwerveDrive swerveDrive) {
        m_swerveDrive = swerveDrive;
        addRequirements(m_swerveDrive);
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

        // create vector reprersents (drivex, drivey) in polar coors; rotate theta by
        // yaw degrees,
        // then convert back to cartesian coords
        if (m_swerveDrive.getDegress() < 26214)
            m_swerveDrive.driveTrain(0, 0, -0);
        else
            m_swerveDrive.driveTrain(0, 0, 0);
        SmartDashboard.putNumber(
            "Gyro Displacement Y",
            m_swerveDrive.getGyroDisplacementY()
        );

        SmartDashboard.putNumber(
            "Gyro Displacement X",
            m_swerveDrive.getGyroDisplacementX()
        );
        SmartDashboard.putNumber(
            "Gyro Displacement Z",
            m_swerveDrive.getGyroDisplacementUseless()
        );

        SmartDashboard.putNumber("Rotation", m_swerveDrive.getDegress());
    }

    /**
     * 
     * @param input the joystick value that should have a deadzone
     * @return input if the absoluteValue of input is greater than .1 otherwise 0
     */
    public double deadzone(double input) {
        if (Math.abs(input) > .05)
            return input * Math.abs(input);

        return 0;
    }
}