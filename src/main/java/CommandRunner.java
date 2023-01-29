import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class CommandRunner

{
  CommandScheduler scheduler = CommandScheduler.getInstance();
  public void runCommands()
  {
    scheduler.run();

  }

}
