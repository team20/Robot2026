package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.ClampedP;
import frc.robot.ClampedP.ClampedPConstants;
import frc.robot.subsystems.AngularPositionSubsystem;

public class AngularPositionCommands {

	public static Command RunToAngleHardware(AngularPositionSubsystem subsystem, double angle, double tolerance) {
		return new SequentialCommandGroup(
				new SetAngleHardware(subsystem, angle),
				new SettleAngle(subsystem, tolerance));
	}

	public static class SetAngleHardware extends Command {
		private final AngularPositionSubsystem m_subsystem;
		private final double m_angle;

		public SetAngleHardware(AngularPositionSubsystem subsystem, double angle) {
			m_subsystem = subsystem;
			m_angle = angle;
			setName(String.format("Set %s Angle Using Motor Controller", m_subsystem.getName()));
			addRequirements(m_subsystem);
		}

		// Called every time the scheduler runs while the command is scheduled.
		@Override
		public void initialize() {
			m_subsystem.moveToPosition(m_angle);
		}

		// Called once the command ends or is interrupted.
		@Override
		public void end(boolean interrupted) {
			// Don't stop motor
		}

		// Returns true when the command should end.
		@Override
		public boolean isFinished() {
			return true;
		}
	}

	public static class SettleAngle extends Command {
		private final AngularPositionSubsystem m_subsystem;
		private final double m_tolerance;

		public SettleAngle(AngularPositionSubsystem subsystem, double tolerance) {
			m_subsystem = subsystem;
			m_tolerance = tolerance;
			setName(String.format("Settle %s Angle", m_subsystem.getName()));
			addRequirements(m_subsystem);
		}

		// Called every time the scheduler runs while the command is scheduled.
		@Override
		public void initialize() {

		}

		// Called once the command ends or is interrupted.
		@Override
		public void end(boolean interrupted) {

		}

		// Returns true when the command should end.
		@Override
		public boolean isFinished() {
			return m_subsystem.isSettled(m_tolerance);
		}
	}

	public static class RunToAngleSoftware extends Command {
		private final AngularPositionSubsystem m_subsystem;
		private final ClampedPConstants m_constants;
		private final double m_angle;

		public RunToAngleSoftware(AngularPositionSubsystem subsytem, double angle, ClampedPConstants constants) {
			m_subsystem = subsytem;
			m_angle = angle;
			m_constants = constants;
			setName(String.format("Run %s To Angle Using PID Controller", m_subsystem.getName()));
			addRequirements(m_subsystem);
		}

		@Override
		public void initialize() {
			m_subsystem.setSetpoint(m_angle);
		}

		// Called every time the scheduler runs while the command is scheduled.
		@Override
		public void execute() {
			double error = m_subsystem.getPosition() - m_angle;
			double power = ClampedP.clampedP(
					error, m_constants.minPower(), m_constants.maxPower(), m_constants.maxErr(),
					m_constants.tolerance());
			m_subsystem.runAtDutyCycle(power);
		}

		// Called once the command ends or is interrupted.
		@Override
		public void end(boolean interrupted) {
			m_subsystem.stop();
		}

		// Returns true when the command should end.
		@Override
		public boolean isFinished() {
			return m_subsystem.isSettled(m_constants.tolerance());
			// return AngleUtility.minDifference(m_subsystem.getPosition(), m_angle) <
			// m_constants.tolerance();
		}
	}

	public static class RunAtPower extends Command {
		private final AngularPositionSubsystem m_subsystem;
		private final Timer m_timer;
		private final double m_time;
		private final double m_speed;

		public RunAtPower(AngularPositionSubsystem subsystem, double speed, double time) {
			m_subsystem = subsystem;
			m_timer = new Timer();
			m_time = time;
			m_speed = speed;
			setName(String.format("Run %s At Power", m_subsystem.getName()));
			addRequirements(m_subsystem);
		}

		// Called when the command is initially scheduled.
		@Override
		public void initialize() {
			m_timer.reset();
			m_timer.start();
			m_subsystem.runAtDutyCycle(m_speed);
		}

		// Called once the command ends or is interrupted.
		@Override
		public void end(boolean interrupted) {
			m_timer.stop();
			m_subsystem.stop();
		}

		// Returns true when the command should end.
		@Override
		public boolean isFinished() {
			return m_time > 0 && m_timer.hasElapsed(m_time);
		}
	}

	public static class RunAtPowerSignal extends Command {
		private final DoubleSupplier m_speed;
		private final AngularPositionSubsystem m_subsystem;
		private final ClampedPConstants m_constants;

		public RunAtPowerSignal(AngularPositionSubsystem subsystem, DoubleSupplier speed, ClampedPConstants constants) {
			m_subsystem = subsystem;
			m_speed = speed;
			m_constants = constants;
			setName(String.format("Run %s At Power From Trigger", m_subsystem.getName()));
			addRequirements(m_subsystem);
		}

		// Called every time the scheduler runs while the command is scheduled.
		@Override
		public void execute() {
			m_subsystem.runAtDutyCycle(MathUtil.applyDeadband(m_speed.getAsDouble(), m_constants.tolerance()));
		}

		// Called once the command ends or is interrupted.
		@Override
		public void end(boolean interrupted) {
			m_subsystem.stop();
		}

		// Returns true when the command should end.
		@Override
		public boolean isFinished() {
			return false;
		}
	}
}