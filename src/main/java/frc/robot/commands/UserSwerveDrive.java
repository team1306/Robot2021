/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;

import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utils.UserAnalog;
import frc.robot.utils.UserDigital;

/**
 * The UserSwerveDrive command uses the SwerveDrive command to execute
 * actions and gather data
 */
public class UserSwerveDrive extends CommandBase {
    public final SwerveDrive m_swerveDrive;
    private final UserAnalog driveX;
    private final UserAnalog driveY;
    private UserAnalog turn;
    private final UserAnalog turnLeft;
    private final UserAnalog turnRight;
    private final UserDigital reset;

    /**
     * Default constructor for UserSwerveDrive. Passes a SwerveDrive
     * object and assigns the UserAnalog controls to each variable.
     * 
     * @param m_swerveDrive
     * @param driveY        analog input for up/down movement
     * @param driveX        analog input for left/right movement
     * @param turn          analog input for turning
     */
    public UserSwerveDrive(SwerveDrive m_swerveDrive, UserAnalog driveX, UserAnalog driveY, UserAnalog turnRight, UserAnalog turnLeft, UserDigital reset) {
        this.m_swerveDrive = m_swerveDrive;
        this.addRequirements(m_swerveDrive);
        this.turnRight = turnRight;
        this.turnLeft = turnLeft;
        turn = turnRight;
        this.reset = reset;
        this.driveX = driveX;
        this.driveY = driveY;
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
        this.turn = turnLeft.get() > turnRight.get() ? turnLeft : turnRight;
        
        double turnTarget = turnRight.get() - turnLeft.get();
        turnTarget = deadzone(turnTarget);
        
        double driveXTarget = deadzone(driveX.get());
        double driveYTarget = deadzone(driveY.get());

        m_swerveDrive.driveTrain(driveXTarget * Constants.FASTEST_SPEED_METERS, 
                                     - driveYTarget * Constants.FASTEST_SPEED_METERS, 
                                      turnTarget * Constants.FASTEST_ANGULAR_VELOCITY * 5);

        if(reset.get()) {
            m_swerveDrive.resetAllWheels();
        }
        // getting data to put onto shuffleboard 
        smartdashboard();
    }

    /**
     * 
     * @param input the joystick value that should have a deadzone
     * @return input if the absoluteValue of input is greater than .1
     *          otherwise 0
     */
    public double deadzone(double input) {
        if(Math.abs(input) > .1)
            return input;

        return 0;
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

    public void setModuleStates(SwerveModuleState[] states) {
        m_swerveDrive.setModuleStates(states);
    }

    public void smartdashboard() {
        m_swerveDrive.frontLeft.shuffleboard("Front Left");
        m_swerveDrive.backLeft.shuffleboard("Back Left");
        m_swerveDrive.frontRight.shuffleboard("Front Right");
        m_swerveDrive.backRight.shuffleboard("Back Right");
        SmartDashboard.putNumber("X Joystick Value", driveX.get());
        SmartDashboard.putNumber("Y Joystick Value", driveY.get());
        SmartDashboard.putNumber("Turn Value", turn.get());
    }
    

}