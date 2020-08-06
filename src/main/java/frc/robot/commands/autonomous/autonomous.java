package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.autonomous.MoveOffLine;

public class Autonomous extends CommandGroup {
    public Autonomous() {
        addSequential(new MoveOffLine(1.0)); // temp time
        addSequential(new TurnAround());
    }
}