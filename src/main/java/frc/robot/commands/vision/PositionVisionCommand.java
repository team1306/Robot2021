package frc.robot.commands.vision;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.EntryNotification;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.NetworkTablePaths;

public class PositionVisionCommand extends CommandBase {

    private final DriveTrain driveTrain;
    private final Shooter shooter;

    private final NetworkTableEntry angleEntry;

    private final double period = 5; // 10 ms
    private final double velocityTolerance = 0.1;
    private final double positionTolerance = 0.75;

    private PositionVisionPIDController angleFollower;
    private final double kP = 0.0004;
    private final double kI = 0.00000000000;
    private final double kD = 0.000001;

    private boolean finished = false;

    public PositionVisionCommand(Shooter shooter, DriveTrain driveTrain) {
        this.driveTrain = driveTrain;
        this.shooter = shooter;

        NetworkTableInstance ntInst = NetworkTableInstance.getDefault();
        angleEntry = ntInst.getEntry(NetworkTablePaths.shooterAngle);
        angleEntry.addListener(this::listenAngle, EntryListenerFlags.kUpdate);

        angleFollower = new PositionVisionPIDController(kP, kI, kD, period, driveTrain);
    }

    @Override
    public void initialize() {
        angleFollower.start();
        finished = false;

        driveTrain.shift(DriveTrain.K_LOW_GEAR);
    }

    @Override
    public void execute() {
        // turning is on it's own loop, as is robot heading
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

}