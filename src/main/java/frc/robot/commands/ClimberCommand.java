
package frc.robot.commands;

import frc.robot.subsystems.Climber;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An example command that uses an example subsystem.
 */
public class ClimberCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Climber m_climber;
  private final DigitalInput unfold;
  private final DigitalInput isExtendPressed;
  private final DigitalInput isRetractPressed;

  /**
   * Creates a new ClimberCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ClimberCommand(Climber m_subsystem, DigitalInput fold, DigitalInput extend, DigitalInput retract) {
    m_climber = m_subsystem;
    unfold = fold;
    isExtendPressed = extend;
    isRetractPressed = retract;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_climber); 
  }

  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(unfold.get()) {
      m_climber.unfold();
    } else if(isExtendPressed.get()) {
      m_climber.extend();
    } else if(isRetractPressed.get()) {
      m_climber.retract();
    } else {
      m_climber.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climber.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}