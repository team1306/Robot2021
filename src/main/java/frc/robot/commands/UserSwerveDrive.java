/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utils.UserAnalog;

/**
 * The UserSwerveDrive command uses the SwerveDrive command to execute
 * actions and gather data
 */
public class UserSwerveDrive extends CommandBase {
    private final SwerveDrive m_swerveDrive;
    private final UserAnalog driveX;
    private final UserAnalog driveY;
    private final UserAnalog turn;

    /**
     * Default constructor for UserSwerveDrive. Passes a SwerveDrive
     * object and assigns the UserAnalog controls to each variable.
     * 
     * @param m_swerveDrive
     * @param driveY        analog input for up/down movement
     * @param driveX        analog input for left/right movement
     * @param turn          analog input for turning
     */
    public UserSwerveDrive(SwerveDrive m_swerveDrive, UserAnalog driveY, UserAnalog driveX, UserAnalog turn) {
        this.m_swerveDrive = m_swerveDrive;
        this.addRequirements(m_swerveDrive);
        this.driveX = driveX;
        this.driveY = driveY;
        this.turn = turn;
        this.m_swerveDrive.setDefaultCommand(this);
    }

    /**
     * Called when the command is initially scheduled.
     */
    @Override
    public void initialize() {
    }

    /**
     * Called every time the scheduler runs while the command is scheduled.
     */
    @Override
    public void execute() {
        m_swerveDrive.driveTrain(driveX.get() * Constants.FASTEST_SPEED_METERS, driveY.get() * Constants.FASTEST_SPEED_METERS, turn.get() * Constants.FASTEST_ANGULAR_VELOCITY);

        // getting data to put onto shuffleboard
        m_swerveDrive.frontLeft.shuffleboard("Front Left");
        m_swerveDrive.backLeft.shuffleboard("Back Left");
        m_swerveDrive.frontRight.shuffleboard("Front Right");
        m_swerveDrive.backRight.shuffleboard("Back Right");

    }

    /**
     * Called once the command ends or is inturrupted.
     */
    @Override
    public void end(boolean interrupted) {
        m_swerveDrive.driveTrain(0, 0, 0);
    }

    /**
     * Returns true when the command should end.
     */
    @Override
    public boolean isFinished() {
        return false;
    }
}