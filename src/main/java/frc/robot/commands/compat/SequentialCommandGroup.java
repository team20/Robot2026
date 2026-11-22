package frc.robot.commands.compat;

/**
 * Commands v2's {@code SequentialCommandGroup}: runs the commands one after another. Same as
 * {@link Commands#sequence}, including its v3 timing between steps.
 */
public class SequentialCommandGroup extends WrapperCommand {
	public SequentialCommandGroup(org.wpilib.command3.Command... commands) {
		super(Commands.sequence(commands));
	}
}
