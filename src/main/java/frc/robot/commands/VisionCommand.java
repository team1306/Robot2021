package frc.robot.commands;

import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;

public class VisionCommand extends CommandBase {

    private final DriveTrain driveTrain;
    private final Shooter shooter;

    public VisionCommand(Shooter shooter, DriveTrain driveTrain) {
        this.driveTrain=driveTrain;
        this.shooter=shooter;

    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    private void turnByAngle(double radians) {
        DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Constants.K_TRACK_WIDTH_METERS);
        //kinematics works for speeds, but by inputing a position a position can also be yielded
        DifferentialDriveWheelSpeeds turnPositions = kinematics.toWheelSpeeds(new ChassisSpeeds(0, 0, radians));
        driveTrain.positionDrive(driveTrain.metersToRotations(turnPositions.rightMetersPerSecond),driveTrain.metersToRotations(turnPositions.leftMetersPerSecond));
    }
}