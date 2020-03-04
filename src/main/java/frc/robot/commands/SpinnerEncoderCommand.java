package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Spinner;
import frc.robot.utils.Encoder;

public class SpinnerEncoderCommand extends CommandBase{

    private static final Encoder encoder = Encoder.VersaPlanetary;
    private double pulseOffset;

    private double encoderGoal;

    private Spinner spinner;

    public SpinnerEncoderCommand(double rotations, Spinner spinner){
        this.spinner = spinner;
        this.addRequirements(spinner);

        pulseOffset = encoder.rotationsToPulses(rotations);
    }

    @Override
    public void initialize() {
        encoderGoal=pulseOffset+spinner.getEncoderPosition();
    }

    @Override
    public void execute() {
        if(spinner.getEncoderPosition()<encoderGoal){
            spinner.spin(1);
        }else{
            spinner.spin(-1);
        }
    }

    @Override
    public boolean isFinished() {
        return (Math.abs(spinner.getEncoderPosition()-encoderGoal)<10);
    }

}