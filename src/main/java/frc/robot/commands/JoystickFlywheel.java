package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.UserAnalog;

public class JoystickFlywheel extends CommandBase {

    private Shooter shooter;
    private UserAnalog input;

    private final double maxRPM=2000;
    
    public JoystickFlywheel(Shooter shooter, UserAnalog outputVelocity) {
        this.shooter = shooter;
        input = outputVelocity;

        addRequirements(shooter);
        shooter.setDefaultCommand(this);
    }

    @Override
    public void initialize() {
        SmartDashboard.putNumber("kP", shooter.controller.getP());
    }

    @Override
    public void execute() {
        shooter.spinToRPM(input.get()*maxRPM);
        shooter.setKickerPercent(input.get());
        SmartDashboard.putNumber("Spinner RPM", shooter.getRPM());
        double sP = shooter.controller.getP();
        double p = SmartDashboard.getNumber("kP", sP);
        if(p!=sP){
            shooter.controller.setP(p);
        }
        System.out.println(shooter.flywheel.getIdleMode().toString());
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