package frc.robot.commands.vision;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.EntryNotification;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;

public class PositionVisionCommand extends CommandBase {

    private final DriveTrain driveTrain;
    private final Shooter shooter;
    private final NetworkTableEntry angleEntry;
    private int angleListenerHandle;
    private final NetworkTableEntry distanceEntry;
    private int distanceListenerHandle;
    private boolean turning = false;
    private double distance;
    private double angle;

    public PositionVisionCommand(Shooter shooter, DriveTrain driveTrain) {
        this.driveTrain = driveTrain;
        this.shooter = shooter;

        NetworkTableInstance ntInst = NetworkTableInstance.getDefault();
        NetworkTable table = ntInst.getTable("Vision");
        angleEntry = table.getEntry("angle");
        angleListenerHandle=angleEntry.addListener(this::listenAngle, EntryListenerFlags.kUpdate);
        distanceEntry = table.getEntry("distance");
        distanceListenerHandle=distanceEntry.addListener(this::listenDistance, EntryListenerFlags.kUpdate);

    }

    @Override
    public void initialize() {
        angleListenerHandle=angleEntry.addListener(this::listenAngle, EntryListenerFlags.kUpdate);
        distanceListenerHandle=distanceEntry.addListener(this::listenDistance, EntryListenerFlags.kUpdate);    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interupted){
        angleEntry.removeListener(angleListenerHandle);
        distanceEntry.removeListener(distanceListenerHandle);
    }

    @Override
    public boolean isFinished() {
        return !turning && angle<=0.01;
    }

    private void listenAngle(EntryNotification n){
        angle=n.value.getDouble();
        if(!turning && angle>0.01){
            turnByAngle(angle);
        }
    }

    private void listenDistance(EntryNotification n){
        distance = n.value.getDouble();
    }

    private void turnByAngle(double radians) {
        DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Constants.K_TRACK_WIDTH_METERS);
        // kinematics works for speeds, but by inputing a position a position can also
        // be yielded
        DifferentialDriveWheelSpeeds turnPositions = kinematics.toWheelSpeeds(new ChassisSpeeds(0, 0, radians));
        driveTrain.positionDrive(driveTrain.metersToRotations(turnPositions.rightMetersPerSecond),
                driveTrain.metersToRotations(turnPositions.leftMetersPerSecond));
        turning=true;
    }
}