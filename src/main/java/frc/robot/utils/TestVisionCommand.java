package frc.robot.utils;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TestVisionCommand extends CommandBase{

    private NetworkTableEntry sourceEntry = SmartDashboard.getEntry("Heading");
    private NetworkTableEntry destEntry =  NetworkTableInstance.getDefault().getEntry(NetworkTablePaths.shooterAngle);

    private NetworkTableEntry sourceEntryDist = SmartDashboard.getEntry("Distance");
    private NetworkTableEntry destEntryDist =  NetworkTableInstance.getDefault().getEntry(NetworkTablePaths.shooterDistance);


    public TestVisionCommand(){
        sourceEntry.setDouble(0);
        sourceEntryDist.setDouble(0);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute(){
        destEntry.setDouble(sourceEntry.getDouble(0));
        destEntryDist.setDouble(sourceEntryDist.getDouble(0));

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}