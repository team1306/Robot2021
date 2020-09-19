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

    public IntakeCommand(Intake intake, UserAnalog grabberSpeed) {
        this.intake = intake;
        this.grabberSpeed = grabberSpeed;

        addRequirements(intake);
        intake.setDefaultCommand(this);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        /*if (intake.getSwitchTop()) {
            intake.index(-indexSpeed / 2);
        } else if (intake.getSwitchBottom() || indexOverride.get()) {
            intake.index(indexSpeed);
        } else {
            intake.index(0);
        }*/
        double speed = intakeSpeedMultiplier * grabberSpeed.get();
        
        intake.spin(speed);
    }
}