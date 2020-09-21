
package frc.robot.commands;

import frc.robot.subsystems.Index;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utils.UserAnalog;
import frc.robot.utils.UserDigital;

/**
 * An example command that uses an example subsystem.
 */
public class IndexCommand extends CommandBase {
  private final UserDigital isPressed;
  private final Index index;

  /**
   * Creates a new IndexCommand.
   *
   * @param index The subsystem used by this command.
   * @param isPressed whether the spinner should rotate
   */
  public IndexCommand(UserDigital isPressed, Index index) {
    this.isPressed = isPressed;
    this.index = index;
    // Use addRequirements() here to declare subsystem dependencies.
  }

   /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(isPressed.get()) {
      index.rotate();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}