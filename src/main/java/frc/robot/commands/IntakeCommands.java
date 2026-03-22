package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.AngleUtility;
import frc.robot.Constants.Subsystems.IntakeConstants;
import frc.robot.subsystems.IntakeArm;
import frc.robot.subsystems.IntakeWheels;
import frc.robot.subsystems.PositionControlSubsystem;

public class IntakeCommands {
	public static class Teletake extends Command {
		private boolean m_on = false;
		private double m_speed;

		public Teletake(double speed, Trigger trigger) {
			setName("Teleop intake");
			m_speed = speed;
			trigger.debounce(.05).onTrue(Commands.runOnce(() -> m_on ^= true));
			addRequirements(IntakeWheels.getIntakeWheels());
		}

		@Override
		public void execute() {
			if (m_on) {
				IntakeWheels.setWheelPower(m_speed);
			} else {
				IntakeWheels.stopWheel();
			}
		}

		@Override
		public void end(boolean interrupted) {
			if (!interrupted) {
				m_on = false;
			}
			IntakeWheels.stopWheel();
		}
	}

	public static class Spintake extends Command {
		private double m_speed;

		public Spintake(double speed) {
			setName("Spin Intake");
			m_speed = speed;
			addRequirements(IntakeWheels.getIntakeWheels());
		}

		@Override
		public void initialize() {
			IntakeWheels.setWheelPower(m_speed);
		}

		@Override
		public void end(boolean interrupted) {
			IntakeWheels.stopWheel();
		}
	}

	public static class StopIntake extends Command {
		public StopIntake() {
			setName("Stop Intake");
			addRequirements(IntakeArm.getIntakeArm());
		}

		@Override
		public void initialize() {
			IntakeWheels.stopWheel();
		}

		@Override
		public boolean isFinished() {
			return true;
		}
	}

	public static class IntakeToPositionHardware extends Command {
		private final PositionControlSubsystem m_subsystem;
		private final double m_tolerance;
		private final double m_position;

		public IntakeToPositionHardware(PositionControlSubsystem subsystem, double position, double tolerance) {
			m_subsystem = subsystem;
			m_position = position;
			m_tolerance = tolerance;
			setName(String.format("Run %s To Angle Using Motor Controller", m_subsystem.getName()));
			addRequirements(m_subsystem);
		}

		// Called every time the scheduler runs while the command is scheduled.
		@Override
		public void initialize() {
			m_subsystem.setMotorPosition(m_position);
		}

		// Called once the command ends or is interrupted.
		@Override
		public void end(boolean interrupted) {
			m_subsystem.stopMotor();
		}

		// Returns true when the command should end.
		@Override
		public boolean isFinished() {
			return AngleUtility.minDifference(m_subsystem.getMotorRotations(), m_position) < m_tolerance;
		}
	}

	public static Command getArmOutCommand() {
		// return new
		// PositionControlCommands.RunToPositionHardware(IntakeArm.getIntakeArm(),
		// IntakeConstants.kOutPosition,
		// 1);
		return new PositionControlCommands.MoveMotorToPosition(IntakeArm.getIntakeArm(),
				IntakeConstants.kOutPosition,
				0.1, 1.0,
				15, .125, false).withTimeout(2);
	}

	public static Command getArmOutCombinedCommand() {
		return Commands.parallel(
				getArmOutCommand(),
				new IntakeCommands.Spintake(IntakeConstants.kWheelPower));
	}

	public static Command getArmInCombinedCommand() {
		return Commands.sequence(
				new IntakeCommands.StopIntake(),
				IntakeCommands.getInCommand());
	}

	public static Command getInCommand() {
		// return new
		// PositionControlCommands.RunToPositionHardware(IntakeArm.getIntakeArm(),
		// IntakeConstants.kInPosition,
		// 1);
		return new PositionControlCommands.MoveMotorToPosition(IntakeArm.getIntakeArm(),
				IntakeConstants.kInPosition,
				0.1, 1.0,
				15, .125, false).withTimeout(2);
	}

	public static Command getRunArmAtPowerCommand(double power) {
		return new PositionControlCommands.SpinMotorPower(IntakeArm.getIntakeArm(), power);
	}

	public static Command getArmZeroCommand() { // TODO: Find actual power and time needed
		return PositionControlCommands.getZeroCommand(
				IntakeArm.getIntakeArm(),
				-.15 /* POWER */,
				6.7 /* TIME */);
	}

	public static Command getEncoderResetCommand() {
		return new PositionControlCommands.ResetEncoder(IntakeArm.getIntakeArm(), 10);
	}
}