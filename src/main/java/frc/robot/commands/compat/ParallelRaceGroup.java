package frc.robot.commands.compat;

/**
 * Commands v2's {@code ParallelRaceGroup}: runs the commands at the same time and ends when the
 * first one ends, interrupting the others. Same as {@link Commands#race}.
 */
public class ParallelRaceGroup extends WrapperCommand {
	public ParallelRaceGroup(org.wpilib.command3.Command... commands) {
		super(Commands.race(commands));
	}
}
