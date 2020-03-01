package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Spinner;
import frc.robot.utils.UserDigital;

public class SpinnerCommand extends CommandBase {

    private Spinner spinner;
    private UserDigital input;

    public SpinnerCommand(Spinner subsystem, UserDigital isSpinning) {
        spinner = subsystem;
        input = isSpinning;
        addRequirements(subsystem);
        spinner.setDefaultCommand(this);
    }

    @Override
    public void execute() {
        if(input.get()){
            spinner.spin(1);
        }else{
            spinner.spin(0);
        }
    }

}