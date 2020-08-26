package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.autonomous.MoveOffLine;

public class Autonomous extends CommandGroup {

    /**
     * Creates default Autonomous object which adds actions to be done
     * in autonomous in order
     * 
     * TODO figure out why MoveOffLine doesn't show up as a command
     */
    public Autonomous() {
        addSequential(new MoveOffLine(1.0)); // temp time
        addSequential(new TurnAround());
    }
}