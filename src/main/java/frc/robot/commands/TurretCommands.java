package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ClampedP;
import frc.robot.Constants.Subsystems.TurretConstants;
import frc.robot.subsystems.Turret;

public class TurretCommands {
	public static class RunToAngleHardware extends Command {

		private final double m_angle;

		/** Creates a new RunTurretAtSpeed. */
		public RunToAngleHardware(double angle) {
			setName("Run Turret To Angle Using Motor Controller");
			addRequirements(Turret.getTurret());
			m_angle = angle;
		}

		// Called every time the scheduler runs while the command is scheduled.
		@Override
		public void initialize() {
			Turret.setAngle(m_angle);
		}

		// Called once the command ends or is interrupted.
		@Override
		public void end(boolean interrupted) {
			Turret.stop();
		}

		// Returns true when the command should end.
		@Override
		public boolean isFinished() {
			//return false;
			return Math.abs(Turret.getPosition() - m_angle) < TurretConstants.kTolerance;
		}
	}

	/**
	 * Used to control the turret with joystick direction (not magnitude).
	 */
	public static class RunToAngleHardwareSignal extends Command {

		private final DoubleSupplier m_x;
		private final DoubleSupplier m_y;

		public RunToAngleHardwareSignal(DoubleSupplier x, DoubleSupplier y) {
			setName("Run Turret To Dynamic Angle Using Motor Controller");
			addRequirements(Turret.getTurret());
			m_x = x;
			m_y = y;
		}

		// Called every time the scheduler runs while the command is scheduled.
		@Override
		public void execute() {
			double x = m_x.getAsDouble();
			double y = m_y.getAsDouble();
			if (Math.hypot(x, y) > TurretConstants.kLargeDeadzone) {
				double angle = Units.radiansToDegrees(Math.atan2(y, x)) + 180;
				Turret.setAngle(angle);
			}
		}

		// Called once the command ends or is interrupted.
		@Override
		public void end(boolean interrupted) {
			Turret.stop();
		}

		// Returns true when the command should end.
		@Override
		public boolean isFinished() {
			return false;
		}
	}

	public static class RunToAngleSoftware extends Command {
		private final double m_angle;

		public RunToAngleSoftware(double angle) {
			setName("Run Turret To Angle Using PID Controller");
			addRequirements(Turret.getTurret());
			m_angle = angle;
		}

		// Called every time the scheduler runs while the command is scheduled.
		@Override
		public void execute() {
			double error = Turret.getPosition() - m_angle;
			double power = ClampedP.clampedP(
					error, TurretConstants.kMinPower, TurretConstants.kMaxPower, TurretConstants.kMaxErr,
					TurretConstants.kTolerance);
			Turret.runAtDutyCycle(power);
		}

		// Called once the command ends or is interrupted.
		@Override
		public void end(boolean interrupted) {
			Turret.stop();
		}

		// Returns true when the command should end.
		@Override
		public boolean isFinished() {
			return Math.abs(Turret.getPosition() - m_angle) < TurretConstants.kTolerance;
		}
	}

	public static class RunAtPower extends Command {
		private Timer m_timer;
		private double m_time;
		private double m_speed;

		public RunAtPower(double speed, double time) {
			setName("Run Turret At Power");
			addRequirements(Turret.getTurret());
			m_timer = new Timer();
			m_time = time;
			m_speed = speed;
		}

		// Called when the command is initially scheduled.
		@Override
		public void initialize() {
			m_timer.reset();
			m_timer.start();
		}

		@Override
		public void execute() {
			Turret.runAtDutyCycle(m_speed);
		}

		// Called once the command ends or is interrupted.
		@Override
		public void end(boolean interrupted) {
			m_timer.stop();
			Turret.stop();
		}

		// Returns true when the command should end.
		@Override
		public boolean isFinished() {
			return m_time > 0 && m_timer.hasElapsed(m_time);
		}
	}

	public static class RunAtPowerSignal extends Command {
		private DoubleSupplier m_speed;

		public RunAtPowerSignal(DoubleSupplier speed) {
			setName("Run Turret At Power From Trigger");
			addRequirements(Turret.getTurret());
			m_speed = speed;
		}

		// Called every time the scheduler runs while the command is scheduled.
		@Override
		public void execute() {
			Turret.runAtDutyCycle(MathUtil.applyDeadband(m_speed.getAsDouble(), TurretConstants.kSmallDeadzone));
		}

		// Called once the command ends or is interrupted.
		@Override
		public void end(boolean interrupted) {
			Turret.stop();
		}

		// Returns true when the command should end.
		@Override
		public boolean isFinished() {
			return false;
		}
	}
}