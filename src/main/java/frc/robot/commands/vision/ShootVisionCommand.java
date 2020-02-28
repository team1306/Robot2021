package frc.robot.commands.vision;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.EntryNotification;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.NetworkTablePaths;

public class ShootVisionCommand extends CommandBase {

    private final Shooter shooter;

    private NetworkTableEntry distanceEntry;

    public ShootVisionCommand(Shooter shooter) {
        this.shooter = shooter;
        distanceEntry = NetworkTableInstance.getDefault().getEntry(NetworkTablePaths.shooterDistance);
    }

    @Override
    public void initialize() {
        distanceEntry.addListener(this::listenDistance, EntryListenerFlags.kLocal+EntryListenerFlags.kUpdate+EntryListenerFlags.kNew);
    }

    @Override
    public void execute(){
    }

    @Override
    public void end(boolean isInterupted){
        shooter.spinToRPM(0);
    }

    private void listenDistance(EntryNotification note){
        double dist = note.value.getDouble();
        shooter.targetDistance(dist);
    }

    @Override
    public boolean isFinished(){
        return false;
    }

}