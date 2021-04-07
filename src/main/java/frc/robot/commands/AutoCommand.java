package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Path;
import java.util.function.DoubleSupplier;
import java.util.function.DoubleConsumer;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;

public class AutoCommand extends ParallelCommandGroup {
    String trajectoryJSON = "paths/YourPath.wpilib.json";
    Trajectory trajectory = new Trajectory();

    public AutoCommand(DriveTrain drive, Intake intake) {
        super();
        
        // current pos - goal pos -> first lambda
        // rightside setspeed var -> second lambda

        PIDCommand leftSide = new PIDCommand(new PIDController(1.0, 0.0, 0.0), () -> { return 2.0; }, 0, (double var) -> { var+=1; }); 
        PIDCommand rightSide = new PIDCommand(new PIDController(1.0, 0.0, 0.0), () -> { return 2.0; }, 0, (double var) -> { var+=1; }); 

        this.addCommands(leftSide, rightSide);
        this.addRequirements(drive, intake);
    }

    @Override
    public void initialize() {
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
        try {
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
            
            RamseteCommand ramseteCommand = new RamseteCommand(
                trajectory,
                m_robotDrive::getPose,
                new RamseteController(Constants.A_kRamseteB, Constants.A_kRamseteZeta),
                new SimpleMotorFeedforward(DriveConstants.ksVolts,
                                           DriveConstants.kvVoltSecondsPerMeter,
                                           DriveConstants.kaVoltSecondsSquaredPerMeter),
                DriveConstants.kDriveKinematics,
                m_robotDrive::getWheelSpeeds,
                new PIDController(Constants.kPDriveVel, 0, 0),
                new PIDController(Constants.kPDriveVel, 0, 0),
                // RamseteCommand passes volts to the callback
                m_robotDrive::tankDriveVolts,
                m_robotDrive
            );
        


        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

interface DoubleGetter{ 
    double get();
}

interface DoubleSetter{
    void set(double val);
}