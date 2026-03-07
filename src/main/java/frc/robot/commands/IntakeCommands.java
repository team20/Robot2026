package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Subsystems.IntakeConstants;
import frc.robot.subsystems.IntakeArm;
import frc.robot.subsystems.IntakeWheels;

public class IntakeCommands {
	public static class SpinIntake extends Command {
		private double m_speed;

		public SpinIntake(double speed) {
			setName("Spin Intake");
			m_speed = speed;
			addRequirements(IntakeArm.getIntakeArm());
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

	public static Command getOutCommand() {
		return new PositionControlCommands.MoveMotorToPosition(IntakeArm.getIntakeArm(), IntakeConstants.kInPosition,
				0.1, 0.7,
				0.5, 0.125, false);
	}

	public static Command getInCommand() {
		return new PositionControlCommands.MoveMotorToPosition(IntakeArm.getIntakeArm(), IntakeConstants.kOutPosition,
				0.1, 0.7,
				0.5, 0.125, false);
	}

	public static Command getRunArmAtPowerCommand(double power) {
		return new PositionControlCommands.SpinMotorPower(IntakeArm.getIntakeArm(), power);
	}

	public static Command getArmZeroCommand() { // TODO: Find actual power and time needed
		return PositionControlCommands.getZeroCommand(
				IntakeArm.getIntakeArm(),
				-.3 /* POWER */,
				5 /* TIME */);
	}
}