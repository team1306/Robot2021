package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Path;
import java.util.function.DoubleSupplier;
import java.util.function.DoubleConsumer;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;

//ParallelCommandGroup
public class AutoCommand extends ParallelRaceGroup {
    String trajectoryJSON = "paths/YourPath.wpilib.json";
    Trajectory trajectory = new Trajectory();
    RamseteCommand ramseteCommand;

    public AutoCommand(DriveTrain drive, Intake intake) {
        super();

        System.out.println("CONSTRUCTOR for AutoCommand IS RUNNING");

        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
        try {
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
            ramseteCommand = new RamseteCommand(
                trajectory,
                drive::getPose,
                new RamseteController(Constants.A_kRamseteB, Constants.A_kRamseteZeta),
                new SimpleMotorFeedforward(Constants.ksVolts,
                                           Constants.kvVoltSecondsPerMeter,
                                           Constants.kaVoltSecondsSquaredPerMeter),
                Constants.kDriveKinematics,
                drive::getWheelSpeeds,
                new PIDController(0.1, 0, 0), // 0.1 was substituted in for Constants.kPDriveVel, which is the P constant of Drive Velocity ?
                new PIDController(0.1, 0, 0), // TODO tune this constant when testing 
                // RamseteCommand passes volts to the callback
                drive::tankDriveVolts,
                drive
            );
        
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        
        // current pos - goal pos -> first lambda
        // rightside setspeed var -> second lambda
        this.addCommands(ramseteCommand, new AutoIntakeCommand(intake));
        this.addRequirements(drive, intake);
    }
}

class AutoIntakeCommand extends CommandBase {
    Intake m_intake;

    public AutoIntakeCommand(Intake m_intake) {
        m_intake = new Intake();
    }

    @Override
    public void execute() {
        m_intake.spin(true, false);
    }
}