
package frc.robot.commands;

import frc.robot.subsystems.Spinner;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.SwerveWheel;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An example command that uses an example subsystem.
 */
public class ClimberCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Spinner m_spinner; 
  private final DigitalInput isPressed;

  /**
   * Creates a new SpinnerCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ClimberCommand(Spinner m_subsystem) {
    m_spinner = m_subsystem;
    isPressed = new DigitalInput(CHANNEL);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_spinner); 
  }

  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    while(isPressed.get()) {
        m_spinner.spin();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // SECOND ARM COMES DOWN
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}