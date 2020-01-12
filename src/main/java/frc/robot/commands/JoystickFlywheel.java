package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.UserAnalog;

public class JoystickFlywheel extends CommandBase {

    private Shooter shooter;
    private UserAnalog input;

    private static final double kP=0.2;
    private static final double kI=0.0001;
    private static final double kD=0;
    private static final boolean swapPhase=false;

    public JoystickFlywheel(Shooter subsystem, UserAnalog outputVelocity) {
        shooter = subsystem;
        input = outputVelocity;

        addRequirements(shooter);
        shooter.setDefaultCommand(this);
    }

    @Override
    public void execute() {
        shooter.setFlywheelPercent(input.get());
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