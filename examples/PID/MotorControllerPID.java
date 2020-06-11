import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.DigitalInput;

class MotorControllerPID extends SubsystemBase {
    MotorControllerPID() {
        // SPARK MAX
            // get objects
            CANSparkMax sparkmax = new CANSparkMax(SPARKMAX_CAN_ID, MotorType.kBrushless);
            CANEncoder encoder = sparkmax.getEncoder();
            CANPIDController controller = sparkmax.getPIDController();
            // Initialize PID
            controller.setP(kP);
            controller.setI(kI);
            controller.setD(kD);
            controller.setFeedbackDevice(encoder);

        // TALON SRX
            //get objects
            TalonSRX talonsrx = new TalonSRX(TALON_CAN_ID);
            talonsrx.config_kP(SLOT_ID, kP);
            talonsrx.config_kI(SLOT_ID, kI);
            talonsrx.config_kD(SLOT_ID, kD);

        // Sparks (regular) and Victor SPX
            //Just dont. You can't on a regular spark, and Talons are faster than victors and we have enough.
    }

    public void GoToPosition(double positionUnits){
        sparkMaxPIDController.setReference(positionUnits, ControlType.kPosition);
        TalonSRX talon;
        talon.set(ControlMode.Position, positionUnits);
    }
}