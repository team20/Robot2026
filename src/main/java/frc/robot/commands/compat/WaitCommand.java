package frc.robot.commands.compat;

/**
 * Commands v2's {@code WaitCommand}: does nothing for a time, then ends. Same as
 * {@link Commands#waitSeconds}.
 */
public class WaitCommand extends WrapperCommand {
	/**
	 * @param seconds how long to wait
	 */
	public WaitCommand(double seconds) {
		super(Commands.waitSeconds(seconds));
	}
}
