package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class AutoVisionCommand extends SequentialCommandGroup{

    public AutoVisionCommand(DriveTrain driveTrain, Shooter shooter, Intake intake){
        VisionCommand visionCommand = new VisionCommand(driveTrain, shooter, intake);
        addCommands(new ParallelDeadlineGroup(new WaitCommand(10),visionCommand), new TimedDriveCommand(1, driveTrain));
    }

}

class TimedDriveCommand extends WaitCommand{

    DriveTrain driveTrain;

    public TimedDriveCommand(double seconds, DriveTrain driveTrain){
        super(seconds);
        this.driveTrain = driveTrain;
        addRequirements(driveTrain);
    }

    @Override
    public void execute() {
        // TODO Auto-generated method stub
        super.execute();
        driveTrain.tankDrive(-0.25, -0.25);
    }
}