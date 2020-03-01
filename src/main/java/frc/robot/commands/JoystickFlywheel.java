package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.UserAnalog;

public class JoystickFlywheel extends CommandBase {

    private Shooter shooter;
    private UserAnalog input;
    
    public JoystickFlywheel(Shooter shooter, UserAnalog outputVelocity) {
        this.shooter = shooter;
        input = outputVelocity;

        addRequirements(shooter);
        shooter.setDefaultCommand(this);
    }

    @Override
    public void execute() {
        shooter.setFlywheelPercent(input.get());
        shooter.setKickerPercent(input.get());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interupted) {
        if (interupted) {
            shooter.stopFlywheel();
        }
    }

   

}