package frc.robot.commands.vision;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.EntryNotification;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.NetworkTablePaths;

public class ShootVisionCommand extends CommandBase {

    private final Shooter shooter;
    private final Intake intake;

    private final double tolerance = 15.0;
    double targetRPM = 0;
    boolean isRunning = false;

    private NetworkTableEntry distanceEntry;

    public ShootVisionCommand(Shooter shooter, Intake intake) {
        this.shooter = shooter;
        this.intake = intake;
        distanceEntry = NetworkTableInstance.getDefault().getEntry(NetworkTablePaths.shooterDistance);
        distanceEntry.addListener(this::listenDistance, EntryListenerFlags.kUpdate);

        this.addRequirements(intake, shooter);
    }

    @Override
    public void initialize() {
        isRunning = true;
    }

    @Override
    public void execute(){
        if(Math.abs(shooter.getRPM()-targetRPM)<tolerance){
            shoot();
        }else{
            stopShoot();
        }
    }

    @Override
    public void end(boolean isInterupted){
        isRunning = false;
        shooter.spinToRPM(0);
        stopShoot();
    }

    private void listenDistance(EntryNotification note){
        double dist = note.value.getDouble();
        if(isRunning){
        targetRPM = shooter.targetDistance(dist);
        System.out.println("Setting Shooter Speed: "+targetRPM);
        }
    }

    @Override
    public boolean isFinished(){
        return false;
    }

    public void shoot(){
        shooter.setKickerPercent(1);
        intake.index(0.7);
    }

    public void stopShoot(){
        shooter.setKickerPercent(0);
        intake.index(0);
    }
}