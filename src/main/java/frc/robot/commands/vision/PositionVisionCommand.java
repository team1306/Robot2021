package frc.robot.commands.vision;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.EntryNotification;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.NetworkTablePaths;

public class PositionVisionCommand extends CommandBase {

    private final DriveTrain driveTrain;
    private final Shooter shooter;

    private final NetworkTableEntry angleEntry;
    private final NetworkTableEntry distanceEntry;
    private final NetworkTableEntry putHeading;

    private final double period = 5; // 10 ms
    private final double velocityTolerance = 0.1;
    private final double positionTolerance = 1;

    private PositionVisionPIDController angleFollower;
    private final double kP = 0.002;
    private final double kI = 0.00001;
    private final double kD = 0.0001;

    private double distance;
    private boolean finished = false;

    public PositionVisionCommand(Shooter shooter, DriveTrain driveTrain) {
        this.driveTrain = driveTrain;
        this.shooter = shooter;

        NetworkTableInstance ntInst = NetworkTableInstance.getDefault();
        angleEntry = ntInst.getEntry(NetworkTablePaths.shooterAngle);
        angleEntry.addListener(this::listenAngle, EntryListenerFlags.kUpdate);
        distanceEntry = ntInst.getEntry(NetworkTablePaths.shooterDistance);
        distanceEntry.addListener(this::listenDistance, EntryListenerFlags.kUpdate);

        putHeading = ntInst.getEntry(NetworkTablePaths.robotHeading);

        angleFollower = new PositionVisionPIDController(kP, kI, kD, period, driveTrain);
    }

    @Override
    public void initialize() {
        angleFollower.start(driveTrain.getHeadingDegrees());
        finished = false;
    }

    @Override
    public void execute() {
        putHeading.setDouble(driveTrain.getHeadingDegrees());
    }

    @Override
    public void end(boolean interupted) {
        angleFollower.stop();
    }

    @Override
    public boolean isFinished() {
        // variable set in listener for implementation convenience
        return finished;
    }

    private void listenAngle(EntryNotification n) {
        double heading = n.value.getDouble();
        angleFollower.setGoalHeading(heading);

        // we are done when not moving and at goal position
        finished = Math.abs(driveTrain.getRotVelocity()) < velocityTolerance
                && Math.abs(heading - driveTrain.getHeadingDegrees()) < positionTolerance;
    }

    private void listenDistance(EntryNotification n) {
        distance = n.value.getDouble();
        shooter.targetDistance(distance);
    }

}