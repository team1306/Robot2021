package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
//import frc.robot.utils.Encoder;
import frc.robot.utils.UserDigital;


public class IntakeCommand extends CommandBase{
    private final UserDigital forward;
    private final UserDigital backward;
    private final Intake m_intake; 

    public IntakeCommand(Intake m_intake, UserDigital forward, UserDigital backward) {
        System.out.println("Code is running in intake");
        this.forward = forward;
        this.backward = backward;
        this.m_intake = m_intake;
        this.addRequirements(m_intake);
        this.m_intake.setDefaultCommand(this);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        m_intake.spin(forward.get(), backward.get());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}