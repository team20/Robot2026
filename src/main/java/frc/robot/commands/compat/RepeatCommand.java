package frc.robot.commands.compat;

/**
 * Commands v2's {@code RepeatCommand}: runs a command, then starts it again each time it ends.
 * Never ends on its own. Same as {@link Commands#repeatingSequence} with one command: at most one
 * pass per loop.
 */
public class RepeatCommand extends WrapperCommand {
	/**
	 * @param command the command to repeat
	 */
	public RepeatCommand(org.wpilib.command3.Command command) {
		super(Commands.repeatingSequence(command));
	}
}
