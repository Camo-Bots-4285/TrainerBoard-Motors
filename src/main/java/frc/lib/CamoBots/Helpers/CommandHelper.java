package frc.lib.CamoBots.Helpers;

import java.util.Set;
import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * CommandHelper
 * 
 * A utility class that safely handles command creation and execution.
 * 
 * Sometimes commands depend on subsystems that might not be available (null),
 * especially when using shared code or different robot configurations.
 * This class provides a safe way to get commands only if they are valid and
 * all required subsystems exist.
 * 
 * If the command or any required subsystem is missing, it returns a no-operation
 * ("do nothing") command to prevent runtime errors or crashes.
 * 
 * Usage example:
 * 
 *   // Safely get a command from a subsystem that might not exist
 *   Supplier<Command> safeCommand = CommandHelper.returnIfValid(() -> someSubsystem.createCommand());
 * 
 *   // Later run the command safely
 *   safeCommand.get();
 * 
 * @author CD
 * @since 2025 FRC Off Season
 */
public class CommandHelper {

  /**
   * Returns a Supplier that provides a valid command if available.
   * 
   * This method:
   * - Checks if the commandSupplier is null.
   * - Evaluates the command using commandSupplier.get().
   * - Checks if the command is null.
   * - Checks if any required subsystem of the command is null.
   * - Catches any exceptions during command creation.
   * 
   * If any check fails, it returns a Supplier that provides a "do nothing" command.
   * This avoids errors from running invalid commands.
   * 
   * @param commandSupplier A Supplier that creates the desired command.
   *                        Example: () -> subsystem.createCommand()
   * @return A Supplier that always provides a valid command or a no-op command.
   */
  public static Command returnIfValid(Supplier<Command> commandSupplier) {
    if (commandSupplier == null) {
        System.err.println("CommandHelper: Supplier is null.");
        return Commands.none();
    }

    try {
        Command cmd = commandSupplier.get();

        if (cmd == null) {
            System.err.println("CommandHelper: Supplied command is null.");
            return Commands.none();
        }

        if (cmd.getRequirements().contains(null)) {
            System.err.println("CommandHelper: Command has null subsystem requirement.");
            return Commands.none();
        }

        return cmd;

    } catch (Exception e) {
        System.err.println("CommandHelper: Exception while getting command: " + e.getMessage());
        return Commands.none();
    }
}

public static Supplier<Command> returnIfValidSupplier(Supplier<Command> commandSupplier) {
  return () -> {
      if (commandSupplier == null) {
          return Commands.none();
      }

      try {
          Command cmd = commandSupplier.get();
          if (cmd == null) {
              System.err.println("CommandHelper: Command was null.");
              return Commands.none();
          }
          if (cmd.getRequirements().contains(null)) {
              System.err.println("CommandHelper: Command has null subsystem requirement!");
              return Commands.none();
          }
          return cmd;
      } catch (Exception e) {
          System.err.println("CommandHelper: Exception creating command: " + e.getMessage());
          return Commands.none();
      }
  };
}

    /**
     * Returns a command that defers to the current state for its behavior.
     * This allows dynamic state-based behavior for buttons without hardcoding to whileTrue.
     *
     * @param commandSupplier Supplier that fetches the command from the current state
     * @return A deferred Command that evaluates the supplier at runtime
     */
    public static Command getStateBoundCommand(Supplier<Command> commandSupplier) {
      return Commands.defer(commandSupplier, Set.of()); // empty Set = no extra requirements
  }

}

