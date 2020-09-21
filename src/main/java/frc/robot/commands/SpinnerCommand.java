
package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Spinner;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An example command that uses an example subsystem.
 */
public class SpinnerCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Spinner m_spinner; 
  private final DigitalInput isSpinnerOn;
  private final DigitalInput isArmExtended;

  /**
   * Creates a new SpinnerCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public SpinnerCommand(Spinner m_subsystem, DigitalInput spinner, DigitalInput arm) {
    m_spinner = m_subsystem;
    this.isSpinnerOn = spinner;
    this.isArmExtended = arm;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_spinner); 
  }

  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(isSpinnerOn.get()) {
      if(m_spinner.getArmAngle() > Constants.K_GOAL_ANGLE) { // write method for arm angle testing
        m_spinner.spin();
      } else {
        m_spinner.extend();
        m_spinner.spin();
      }
    } else if(isArmExtended.get()) {
      m_spinner.extend();
    } else {
      m_spinner.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_spinner.retract();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}