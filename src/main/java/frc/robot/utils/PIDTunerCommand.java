package frc.robot.utils;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.EntryNotification;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PIDTunerCommand extends CommandBase {

    private ArrayList<BaseMotorController> controllers;
    // Fixed variables
    private FeedbackDevice sensorType;
    private ControlMode outputMode;
    private double minOut = -1;
    private double maxOut = 1;
    private final Encoder encoder;
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
    private NetworkTableEntry valueEntry;
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
            FeedbackDevice sensorType, SubsystemBase[] requirements, Encoder encoder, BaseMotorController controller,
            BaseMotorController... controllers) {
        this.outputMode = controlMode;
        this.encoder = encoder;
        this.minOut = minOutput;
        this.maxOut = maxOutput;
        this.swapPhase = invertPhase;
        this.sensorType = sensorType;

        addRequirements(requirements);

        // intialize network table elements and add listeners
        table = Shuffleboard.getTab("Tuner");
        phaseEntry = table.add("SensorPhase", invertPhase).getEntry();
        pEntry = table.add("P-gain", 0).getEntry();
        iEntry = table.add("I-gain", 0).getEntry();
        dEntry = table.add("D-gain", 0).getEntry();
        valueEntry = table.add("GoalValue", 0).getEntry();
        phaseEntry.addListener(this::phaseListener, EntryListenerFlags.kUpdate);
        pEntry.addListener(this::pListener, EntryListenerFlags.kUpdate);
        iEntry.addListener(this::iListener, EntryListenerFlags.kUpdate);
        dEntry.addListener(this::dListener, EntryListenerFlags.kUpdate);
        valueEntry.addListener(this::valueListener, EntryListenerFlags.kUpdate);

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

    /**
     * For single subsystem requirement
     */
    public PIDTunerCommand(ControlMode controlMode, double minOutput, double maxOutput, boolean invertPhase,
            FeedbackDevice sensorType, SubsystemBase requirement, Encoder encoder, BaseMotorController controller,
            BaseMotorController... controllers) {
        this(controlMode, minOutput, maxOutput, invertPhase, sensorType, new SubsystemBase[] { requirement }, encoder,
                controller, controllers);
    }

    /**
     * Initialze the entries with default values and push to networkTables
     */
    private void pushTableEntries() {
        phaseEntry.setBoolean(swapPhase);
        pEntry.setDouble(P);
        iEntry.setDouble(I);
        dEntry.setDouble(D);
    }

    /**
     * Update the graphs of position and velocity for the dashboard for each motor
     */
    private void updateOutputs() {
        for (int i = 0; i < controllers.size(); i++) {
            BaseMotorController c = controllers.get(i);
            NetworkTableEntry posEntry = positionOutputs.get(i);
            NetworkTableEntry velEntry = velocityOutputs.get(i);
            posEntry.setDouble(encoder.pulsesToRotations(c.getSelectedSensorPosition()));
            velEntry.setDouble(encoder.PIDVelocityToRPM(c.getSelectedSensorVelocity()));
        }
    }

    /**
     * Listens to an update to the sensor phase and sets accordingly
     * 
     * @param note supplied by the entry when event triggered
     */
    private void phaseListener(EntryNotification note) {
        swapPhase = note.value.getBoolean();
        for (int i = 0; i < controllers.size(); i++) {
            BaseMotorController c = controllers.get(i);
            c.setSensorPhase(swapPhase);
        }
        System.out.println("Listening to phase entry");

    }

    /**
     * Listenes to the changes in p-value and updates the motor controllers to match
     * 
     * @param note supplied by the entry when update triggered
     */
    private void pListener(EntryNotification note) {
        P = note.value.getDouble();
        for (int i = 0; i < controllers.size(); i++) {
            BaseMotorController c = controllers.get(i);
            c.config_kP(0, P);
        }
        System.out.println("Listening to p entry");
    }

    /**
     * Listens to change in the i-value and updates motor controllers to match.
     * Since accumulations in the integral can cause wild variations when the
     * I-value is changed, clears the accumulator first.
     * 
     * @param note supplied by the entry when update triggered
     */
    private void iListener(EntryNotification note) {
        I = note.value.getDouble();
        for (int i = 0; i < controllers.size(); i++) {
            BaseMotorController c = controllers.get(i);
            c.setIntegralAccumulator(0);
            c.config_kI(0, I);
        }
        System.out.println("Listening to i entry");

    }

    /**
     * Listents to the change in d-value and updates controllers
     * 
     * @param note supplied by the entry when update triggered
     */
    private void dListener(EntryNotification note) {
        D = note.value.getDouble();
        for (int i = 0; i < controllers.size(); i++) {
            BaseMotorController c = controllers.get(i);
            c.config_kD(0, D);
        }
        System.out.println("Listening to d entry");

    }

    /**
     * Listens to the change in the goal output (position or velocity, depending on
     * outputMode) and sets all controllers to match.
     * 
     * @param note supplied by the entry when update triggered
     */
    private void valueListener(EntryNotification note) {
        goalVal = note.value.getDouble();
        if (outputMode.equals(ControlMode.Velocity)) {
            goalVal = encoder.RPMtoPIDVelocity(goalVal);
        } else if (outputMode.equals(ControlMode.Position)) {
            goalVal = encoder.rotationsToPulses(goalVal);
        }

        if (isScheduled()) {
            for (int i = 0; i < controllers.size(); i++) {
                BaseMotorController c = controllers.get(i);
                c.set(outputMode, goalVal);
                c.setIntegralAccumulator(0);
                System.out.println("Setting motor to goal " + goalVal);
            }
        }
        System.out.println("Listening to goal entry");

    }

    /**
     * Sets up the PID settings for a motor controller and adds the relevent
     * listeners, lists, and outputs.
     * 
     * @param c - motor controller to initialize
     */
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

    private int dash = 0;

    public void execute() {
        // every 10 executions update graphs
        if (dash == 0) {
            updateOutputs();
        }
        dash = (dash + 1) % 10;

    }

    public boolean isFinished() {
        return false;
    }
}