package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;

public class practiceAuto extends TimedRobot {
    Command autonomousCommand; // make all four other files into commands

    public void robotInit() {
        //oi = new OI(); // what does this do and why doesn't it work
        autonomousCommand = new autonomous();
    }

    public void autonomousInit() {
        if (autonomousCommand != null) autonomousCommand.start();
    }

    public void autonomousPeriod() {
        Scheduler.getInstance().run();
    }
}