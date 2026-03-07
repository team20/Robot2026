package frc.robot.commands;

import static frc.robot.Constants.Subsystems.ClimberConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Subsystems.ClimberConstants;
import frc.robot.subsystems.Climber;

public class ClimberCommands {
	/*
	 * You should consider using the more terse Command factories API instead
	 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-
	 * command-based.html#defining-commands
	 */

	public static Command getZeroCommand() { // TODO: Find actual power and time needed
		return PositionControlCommands.getZeroCommand(
				Climber.getClimber(),
				-.3 /* POWER */,
				5 /* TIME */);
	}

	public static Command getRunAtPowerCommand(double power) {
		return new PositionControlCommands.SpinMotorPower(Climber.getClimber(), power);
	}

	public static Command getClimbCommand() {
		return new PositionControlCommands.MoveMotorToPosition(Climber.getClimber(), ClimberConstants.kClimbPosition,
				0.1, 0.5,
				0.5, 0.125, false);
	}

	public static Command getRetractCommand() {
		return new PositionControlCommands.MoveMotorToPosition(Climber.getClimber(), ClimberConstants.kRetractPosition,
				0.1,
				0.5, 0.5, 0.125, false);
	}

	public static class RunToHeightSoftware extends Command {
		private double m_height;
		private double m_revolutions;

		public RunToHeightSoftware(double height) {
			setName("Run Climber to Height");
			m_height = height;
			m_revolutions = m_height * kGearRatio;
			addRequirements(Climber.getClimber());
		}

		public void execute() {
		}

		public void end(boolean interrupted) {
		}

		public boolean isFinished() {
			return false;
		}
	}
}
