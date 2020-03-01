package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;
import frc.robot.utils.UserDigital;

public class ClimberCommand extends CommandBase{

    Climber climber;
    UserDigital input;
    UserDigital reverse;
    
    public ClimberCommand(Climber climber, UserDigital isClimbing, UserDigital reverse){
        this.climber= climber;
        this.input = isClimbing;
        this.reverse= reverse;
        addRequirements(climber);
        climber.setDefaultCommand(this);
    }

    @Override
    public void execute() {
        if(input.get()){
            climber.climb(0.8);
        }else if(reverse.get()){
            climber.climb(-0.8);
        } else {
            climber.climb(0);
        }
        
    }

}