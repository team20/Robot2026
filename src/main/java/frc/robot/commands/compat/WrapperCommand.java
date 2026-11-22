package frc.robot.commands.compat;

import org.wpilib.command3.Coroutine;
import org.wpilib.command3.Mechanism;

/**
 * A compat {@link Command} whose body is another v3 command. Name and requirements come from that
 * command. When this wrapper is canceled, the v3 scheduler cancels the inner command too (a child
 * never outlives its parent), and the inner command handles its own cleanup.
 *
 * <p>Used by {@link Command#wrap}, the {@link Commands} factories, and the v2-named group classes
 * ({@link SequentialCommandGroup}, {@link ParallelRaceGroup}, {@link WaitCommand},
 * {@link RepeatCommand}).
 */
class WrapperCommand extends Command {
	private final org.wpilib.command3.Command m_inner;

	WrapperCommand(org.wpilib.command3.Command inner) {
		m_inner = inner;
		setName(inner.name());
		addRequirements(inner.requirements().toArray(Mechanism[]::new));
		// So that e.g. withName() or withTimeout() on a CANCEL_INCOMING command keeps it
		setPriority(inner.priority());
	}

	@Override
	public void run(Coroutine coroutine) {
		coroutine.await(m_inner);
	}
}
