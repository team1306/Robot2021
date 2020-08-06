package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.autonomous.moveOffLine;

public class autonomous extends CommandGroup {
    public autonomous() {
        addSequential(new moveOffLine(1.0)); // temp time
        addSequential(new turnAround());
    }
}