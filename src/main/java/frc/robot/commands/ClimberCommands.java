package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Subsystems.ClimberConstants;
import frc.robot.subsystems.Climber;

public class ClimberCommands {
	public static Command getZeroCommand() { // TODO: Find actual power and time needed
		return PositionControlCommands.getZeroCommand(
				Climber.getClimber(),
				-.3 /* POWER */,
				5 /* TIME */);
	}

	public static Command getResetCommand() {
		return new PositionControlCommands.ResetEncoder(Climber.getClimber(), 0);
	}

	public static Command getRunAtPowerCommand(double power) {
		return new PositionControlCommands.SpinMotorPower(Climber.getClimber(), power);
	}

	public static Command getClimbCommand() {
		return new PositionControlCommands.MoveMotorToPosition(Climber.getClimber(), ClimberConstants.kClimbPosition,
				0.3, 0.9,
				3000, 1000, false);
	}

	public static Command getRetractCommand() {
		return new PositionControlCommands.MoveMotorToPosition(Climber.getClimber(), ClimberConstants.kRetractPosition,
				0.3,
				.9, 3000, 1000, false);
	}
}
