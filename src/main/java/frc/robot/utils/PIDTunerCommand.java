package frc.robot.utils;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class PIDTunerCommand extends CommandBase {

    private ArrayList<BaseMotorController> controllers;
    // Fixed variables
    private FeedbackDevice sensorType;
    private ControlMode outputMode;
    private double minOut = -1;
    private double maxOut = 1;

    // changing variables
    private boolean swapPhase;
    private double P = 0;
    private double I = 0;
    private double D = 0;
    private double goalVal = 0;
    // NetworkTable fields
    private ShuffleboardTab table;
    private NetworkTableEntry phaseEntry;
    private NetworkTableEntry pEntry;
    private NetworkTableEntry iEntry;
    private NetworkTableEntry dEntry;
    private NetworkTableEntry goalEntry;
    private ArrayList<NetworkTableEntry> velocityOutputs;
    private ArrayList<NetworkTableEntry> positionOutputs;

    /**
     * Constructs a new PID tuning tool for the given motor controllers. Populates
     * the Shuffleboard and networktables with variables for goal value, P,I,D,
     * value outputs, and sensor phase.
     * 
     * 
     * @param controlMode - currently one of either Position or Velocity
     * @param minOutput   - the minimum percent output of the motor when going
     *                    either direction. Must be greater or equal to 0
     * @param maxOutput   - the maximum percent output of the motor going either
     *                    direction. Must be greater than minOutput and not greater
     *                    than 1
     * @param invertPhase - Used to control the direction of the senser in relation
     *                    to the output shaft. Set true if positive on the output
     *                    shaft is negative on the sensor, else false.
     * @param sensorType  - the type of the sensor providing feedback. Usually a
     *                    QuadEncoder
     * @param controller  - The motor controller to test the PID on.
     * @param controllers - Option other controllers to test the same PID on
     */
    public PIDTunerCommand(ControlMode controlMode, double minOutput, double maxOutput, boolean invertPhase,
            FeedbackDevice sensorType, BaseMotorController controller, BaseMotorController... controllers) {
        this.outputMode = controlMode;
        this.minOut = minOutput;
        this.maxOut = maxOutput;
        this.swapPhase = invertPhase;
        this.sensorType = sensorType;
        // intialize network table elements
        table = Shuffleboard.getTab("Tuner");
        phaseEntry = table.add("SensorPhase", invertPhase).getEntry();
        pEntry = table.add("P-gain", 0).getEntry();
        iEntry = table.add("I-gain", 0).getEntry();
        dEntry = table.add("D-gain", 0).getEntry();
        goalEntry = table.add("GoalValue", 0).getEntry();
        // Go through each motor controller and set up the tuner
        this.controllers = new ArrayList<BaseMotorController>();
        this.positionOutputs = new ArrayList<NetworkTableEntry>();
        this.velocityOutputs = new ArrayList<NetworkTableEntry>();
        intializeMotorController(controller);
        for (int i = 0; i < controllers.length; i++) {
            intializeMotorController(controllers[i]);
        }
        pushTableEntries();
    }

    private void pushTableEntries() {
        phaseEntry.setBoolean(swapPhase);
        pEntry.setDouble(P);
        iEntry.setDouble(I);
        dEntry.setDouble(D);
        System.out.println("Values Updating");
    }

    private void updateOutputs() {
        for (int i = 0; i < controllers.size(); i++) {
            BaseMotorController c = controllers.get(i);
            NetworkTableEntry posEntry = positionOutputs.get(i);
            NetworkTableEntry velEntry = positionOutputs.get(i);
            posEntry.setDouble(c.getSelectedSensorPosition());
            velEntry.setDouble(c.getSelectedSensorVelocity());
        }
    }

    private void grabUserInputs() {
        P = pEntry.getDouble(0);
        D = dEntry.getDouble(0);
        swapPhase = phaseEntry.getBoolean(swapPhase);
        goalVal = goalEntry.getDouble(0);
        double uI = iEntry.getDouble(0); // Want to be carefull when swapping I due to built up integral values
        if (uI > I) {
            resetIntegrals();
            I = uI;
        }
        updateMotorGains();
    }

    private void updateMotorGains() {
        for (int i = 0; i < controllers.size(); i++) {
            BaseMotorController c = controllers.get(i);
            c.config_kP(0, P);
            c.config_kI(0, I);
            c.config_kD(0, D);
        }
    }

    private void resetIntegrals() {
        for (int i = 0; i < controllers.size(); i++) {
            BaseMotorController c = controllers.get(i);
            // no idea how to actually do this, use a hack
            c.set(ControlMode.PercentOutput, c.getMotorOutputPercent());
        }
    }

    private void intializeMotorController(BaseMotorController c) {
        // remove unwanted extra settings that could cause issues
        PIDSetup.IntializePID(c, 0, 0, 0, minOut, maxOut, sensorType, 0, 0);

        String name = "Motor" + controllers.size();
        controllers.add(c);
        NetworkTableEntry posEntry = table.add(name + "Position", 0).getEntry();
        NetworkTableEntry velEntry = table.add(name + "Velocity", 0).getEntry();
        posEntry.setDouble(0);
        velEntry.setDouble(0);
        velocityOutputs.add(velEntry);
        positionOutputs.add(posEntry);

    }

    int dash = 0;

    public void execute() {
        System.out.println("Executing PID Tuner");
        if (dash == 0) {
            double oldVal = goalVal;
            grabUserInputs();
            updateOutputs();
            if (!(oldVal == goalVal)) {
                for (int i = 0; i < controllers.size(); i++) {
                    BaseMotorController c = controllers.get(i);
                    c.set(outputMode, goalVal);
                }
            }
        }
        dash = (dash + 1) % 10;

    }

    public boolean isFinished() {
        return false;
    }
}