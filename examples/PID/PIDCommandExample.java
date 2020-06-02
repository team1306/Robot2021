import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class PIDCommandExample extends PIDCommand {

    public PIDCommandExample(Subsystem subsystem) {
        super(createPIDController(), 
        // llambda expression for asking the current position
        () -> {
            return getCurrentPosition();
        }, 
        // llamda expression for asking the desired position
        () -> {
            return getGoalPosition();
        },
        // llamda expression with parameter for output of the pid loop
        (double output) -> {
            doSomethingWithOutput(output);
        },
        //list requirements that this command must have.
        subsystem);
    }

    private static PIDController createPIDController() {
        // When looking at the docs, watch out... There are two different
        // PIDControllers!
        // We want the one at edu.wpi.first.wpilibj.controller.PIDController
        // We do NOT want edu.wpi.first.wpilibj.PIDController

        double typicalPeriod = 0.05; // Command based systems refresh every 0.05 seconds
        double Kp = 0.01; // Kp values, the proportional multiplier, tend to be larger.
        double Ki = 0.000001; // Ki values are usually very very very small. This makes sense:
                              // Lots of different terms are being added, so it's multiplying
                              // a very large number. It must be small to have a reasonable
                              // magnitude of effect.
        double Kd = 0.001; // Kd values are usually somewhere in between Kp and Ki.

        // Kp, Ki, Kd, AND period control how your PID controller responds. When you
        // change period, all the other constants change too! These constants are very
        // very hard to determine mathematically, even for industry experts. Instead,
        // many people use a process called tuning. The process is described here:
        // https://www.controldesign.com/articles/2016/how-to-tune-pid-loops/
        // Give it a try on my simulator, https://eganj.github.io/PIDWebsite/

        // Create the PIDController, which will be used in the constructor of the
        // command
        return new PIDController(Kp, Ki, Kd, typicalPeriod);
    }

    // Thats it! That's all we need to make a PIDCommand! However, we could add a few things if
    // we needed to...

    @Override
    public void execute() {
        super.execute(); // This is neccessary for a PIDCommand!

        //do other logic here

    }

    @Override
    public boolean isFinished() {
        return false; // calling super not neccessary here
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted); // This is neccessary for a PIDCommand!

        //Some logic here
    }
}