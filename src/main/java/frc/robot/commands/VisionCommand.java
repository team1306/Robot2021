package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.vision.PositionVisionCommand;
import frc.robot.commands.vision.ShootVisionCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;

public class VisionCommand extends SequentialCommandGroup {
    public VisionCommand(DriveTrain driveTrain, Shooter shooter) {
        super();
        PositionVisionCommand position = new PositionVisionCommand(shooter, driveTrain);
        ShootVisionCommand shoot = new ShootVisionCommand(shooter);
        addCommands(position, shoot);
    }
}