package frc.robot.utils;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;

public class PIDSetup {
    /**
     * 
     * @param c          - the controller to initalize for PID
     * @param kP         - the P-gain coefficient
     * @param kI         - the I-gain coefficient
     * @param kD         - the D-gain coefficient
     * @param minOutput  - The minimum motor output going either direction
     * @param maxOutput  - The maximum motor output going either direction
     * @param sensorType - What sensor is providing the input for the PID
     * @param slotIdx    - 0 for closed loop, 1 for auxilary closed loop
     * @param timeout    - How much time to wait while checking sucess before
     *                   declare error. 0 to skip checking
     */
    public static void IntializePID(BaseMotorController c, double kP, double kI, double kD, double minOutput,
            double maxOutput, FeedbackDevice sensorType, int slotIdx, int timeout) {
        c.configFactoryDefault(timeout);
        c.configSelectedFeedbackSensor(sensorType, slotIdx, timeout);
        /*
        Minimum/Maxiumum outputs:
            "-" = min output, "+" = max output, "|" = 0
        (-1)     <---|+++++++++>(+1) Forward
        (-1)     <---|+++++++++>(+1) Reverse

        So in either case, PeakOutput should be set to max out and Nominal to min out,
        no swapping required.
        */
        c.configNominalOutputForward(minOutput, timeout);// nominal = minimum output
        c.configNominalOutputReverse(minOutput, timeout);
        c.configPeakOutputForward(maxOutput, timeout);// peak = maximum output
        c.configPeakOutputReverse(maxOutput, timeout);

        c.config_kP(slotIdx, kP);
        c.config_kI(slotIdx, kI);
        c.config_kD(slotIdx, kD);
    }

    /**
     * Initializes a spark with the given values for the NEO internal encoder
     */
    public static void IntializePIDSparkNEO(CANSparkMax spark, double kP, double kI, double kD, double maxOutput, double minOutput) {
        CANPIDController c = spark.getPIDController();
        c.setP(kP);
        c.setI(kI);
        c.setD(kD);
        //c.setOutputRange(minOutput, maxOutput);
        c.setFeedbackDevice(spark.getEncoder(EncoderType.kHallSensor, (int) Encoder.NeoInternal.rotationsToPulses(1)));
    }
}