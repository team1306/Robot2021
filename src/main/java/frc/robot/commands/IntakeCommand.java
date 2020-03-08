package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.utils.UserAnalog;
import frc.robot.utils.UserDigital;

public class IntakeCommand extends CommandBase {

    private final double indexSpeed = 0.8;

    private final double intakeSpeedMultiplier = 0.7;

    private final Intake intake;
    private final UserAnalog grabberSpeed;
    private final UserDigital isStuck;
    private final UserDigital indexOverride;

    public IntakeCommand(Intake intake, UserAnalog grabberSpeed, UserDigital isStuck, UserDigital indexOverride) {
        this.intake = intake;
        this.grabberSpeed = grabberSpeed;
        this.isStuck = isStuck;
        this.indexOverride = indexOverride;

        addRequirements(intake);
        intake.setDefaultCommand(this);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if(intake.getSwitchTop()){
            intake.index(-indexSpeed/2);
        }
        else if (intake.getSwitchBottom() || indexOverride.get()) {
            intake.index(indexSpeed);
        } else {
            intake.index(0);
        }
        double speedRight = intakeSpeedMultiplier*grabberSpeed.get();
        double speedLeft = speedRight;
        if(isStuck.get()){
            speedLeft = 0;
        }
        intake.intake(speedRight,speedLeft);
    }
}