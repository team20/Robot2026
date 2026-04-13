package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Subsystems.IntakeConstants;
import frc.robot.subsystems.IntakeArm;
import frc.robot.subsystems.IntakeExtraArm;
import frc.robot.subsystems.IntakeWheels;

public class IntakeCommands {
	private final static double outInMinPower = 0.1;
	private final static double outInMaxPower = 1.0;
	private final static double outInMaxError = 25;
	private final static double outInTolerance = 0.125;

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

	public static class SpinExtraIntake extends Command {
		private double m_speed;

		public SpinExtraIntake(double speed) {
			setName("Spin Extra Intake");
			m_speed = speed;
			addRequirements(IntakeExtraArm.getIntakeExtraArm());
		}

		@Override
		public void initialize() {
			IntakeExtraArm.setMotorPower(m_speed);
		}

		@Override
		public void end(boolean interrupted) {
			IntakeExtraArm.stopMotor();
		}
	}

	public static class StopIntakeWheels extends Command {
		public StopIntakeWheels() {
			setName("Stop Intake Wheels");
			addRequirements(IntakeWheels.getIntakeWheels());
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

	public static class StopExtraIntake extends Command {
		public StopExtraIntake() {
			setName("Stop Extra Intake");
			addRequirements(IntakeExtraArm.getIntakeExtraArm());
		}

		@Override
		public void initialize() {
			IntakeExtraArm.stopMotor();
		}

		@Override
		public boolean isFinished() {
			return true;
		}
	}

	public static Command getArmOutCommand() {
		// return new
		// PositionControlCommands.RunToPositionHardware(IntakeArm.getIntakeArm(),
		// IntakeConstants.kOutPosition,
		// 1);
		return new PositionControlCommands.MoveMotorToPosition(IntakeArm.getIntakeArm(),
				IntakeConstants.kOutPosition,
				outInMinPower, outInMaxPower,
				outInMaxError /* 15 */, outInTolerance, false).withTimeout(2);
	}

	public static Command getArmOutCombinedCommand() {
		return Commands.sequence(
				Commands.race(
						getArmOutCommand(),
						new IntakeCommands.SpinExtraIntake(.65),
						new IntakeCommands.Spintake(IntakeConstants.kWheelPower)),
				new IntakeCommands.Spintake(IntakeConstants.kWheelPower));
	}

	public static Command getArmInCombinedCommand() {
		return Commands.sequence(
				new IntakeCommands.StopIntakeWheels(),
				Commands.parallel(
						IntakeCommands.getInCommand(),
						new IntakeCommands.StopExtraIntake()));
	}

	public static Command getInCommand() {
		// return new
		// PositionControlCommands.RunToPositionHardware(IntakeArm.getIntakeArm(),
		// IntakeConstants.kInPosition,
		// 1);
		return new PositionControlCommands.MoveMotorToPosition(IntakeArm.getIntakeArm(),
				IntakeConstants.kInPosition,
				outInMinPower, outInMaxPower,
				outInMaxError /* 15 */, outInTolerance, false).withTimeout(2);
	}

	public static Command getRunArmAtPowerCommand(double power) {
		return new PositionControlCommands.SpinMotorPower(IntakeArm.getIntakeArm(), power);
	}

	public static Command getArmZeroCommand() {
		return PositionControlCommands.getZeroCommand(
				IntakeArm.getIntakeArm(),
				-.15 /* POWER */,
				6.7 /* TIME */);
	}

	public static Command getEncoderResetCommand() {
		return new PositionControlCommands.ResetEncoder(IntakeArm.getIntakeArm(), 10);
	}
}