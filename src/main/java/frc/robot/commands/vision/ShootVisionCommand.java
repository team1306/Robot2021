package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class ShootVisionCommand extends CommandBase {

    private final Shooter shooter;

    public ShootVisionCommand(Shooter shooter) {
        this.shooter = shooter;
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute(){
    }

    @Override
    public void end(boolean isInterupted){

    }

    @Override
    public boolean isFinished(){
        return false;
    }

}