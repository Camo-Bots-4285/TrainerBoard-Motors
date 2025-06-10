package frc.robot.lib;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;


public class CommandHelper {
  /**
   * Schedule a command only if it exists (is not null).
   * 
   * This method takes a "Supplier" that gives us a command when we ask for it.
   * It then checks if the command is there, and if so, it starts running it.
   * 
   * We use a Supplier to wait until the very last moment before making the command.
   * This is important because the command might depend on a subsystem that might
   * not exist on this robot (it could be null). If the subsystem is missing, the
   * command will be null and won’t run — which keeps the program safe.
   * 
   * @param commandSupplier A way to get the command when needed (could be null if no subsystem) make
   * sure that you call the command like this scheduleIfExists(() -> Example.commmand)
   * 
   */
//     public static void scheduleIfExists(Supplier<Command> commandSupplier) {
//       Command cmd = commandSupplier.get();
//       if (cmd != null) {
//           cmd.schedule();
//       }
//     }

//     public static void scheduleIfValid(Command command) {
//     if (command == null) return;
//     for (Subsystem subsystem : command.getRequirements()) {
//         if (subsystem == null) return;
//     }
//     command.schedule();
// }


/**
 * Get a command only if it is valid and safe to run.
 * 
 * This method takes a "Supplier," which is like a recipe for making a command when we need it.
 * It waits until the very last moment to create the command. This is important because
 * the command might need a subsystem that doesn’t exist on this robot (it could be null).
 * 
 * If the command doesn’t exist or if any subsystem it needs is missing, it returns a "do nothing" command instead.
 * This helps prevent errors or crashes by making sure we only run commands that are safe.
 * 
 * @param commandSupplier A way to get the command when we want it. Example usage:
 *                        getValidCommand(() -> someSubsystem.makeCommand())
 * 
 * @return A valid command to run, or a "do nothing" command if something is missing.
 */
public static Command returnIfValid(Supplier<Command> commandSupplier) {
  Command command = commandSupplier.get();
  if (command == null) return Commands.none();
  for (Subsystem subsystem : command.getRequirements()) {
      if (subsystem == null) return Commands.none();
  }
  return command;
}


}
