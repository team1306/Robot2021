package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.Encoder;
import frc.robot.utils.PIDSetup;

public class Shooter extends SubsystemBase {

    private final TalonSRX flywheel;
    private final Spark kicker;
    private final Encoder flywheelEnc=Encoder.Grayhill256;

    private final int kP = 0;
    private final int kI = 0;
    private final int kD = 0;

    public Shooter() {
        // initalize flywheel for PID
        flywheel = new TalonSRX(Constants.K_SHOOTER_FlYWHEEL_ID);
        PIDSetup.IntializePID(flywheel, kP, kI, kD, 0, 1, FeedbackDevice.QuadEncoder, 0, 0);

        // Intialize other motors
        kicker = new Spark(Constants.K_SHOOTER_KICKER_SPARK);
    }

    public void spinToRPM(double rpm){
        flywheel.set(ControlMode.Velocity, flywheelEnc.RPMtoPIDVelocity(rpm));
    }

    public double getRPM(){
        return flywheelEnc.PIDVelocityToRPM(flywheel.getSelectedSensorVelocity());
    }

    public void stopFlywheel(){
        flywheel.set(ControlMode.PercentOutput, 0);
    }

}