
package frc.robot.commands;

import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utils.UserAnalog;

/**
 * An example command that uses an example subsystem.
 */
public class ShooterCommand extends CommandBase {
  private final UserAnalog speed;
  private final Shooter m_shooter;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ShooterCommand(UserAnalog speed, Shooter shooter) {
    this.speed = speed;
    m_shooter = shooter;
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
    m_shooter.shoot(speed.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.shoot(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}