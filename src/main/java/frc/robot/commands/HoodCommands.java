package frc.robot.commands;

import frc.robot.commands.compat.Command;
import frc.robot.Constants.Subsystems.HoodConstants;
import frc.robot.subsystems.Hood;

public class HoodCommands {
	public static Command getHoodDownCommand() {
		return AngularPositionCommands.RunToAngleHardware(
				Hood.getHood(), HoodConstants.kDownPosition,
				Hood.getConstants().tolerance());
	}

	public static Command getTurnToAngleCommand(double angle) {
		return AngularPositionCommands.RunToAngleHardware(
				Hood.getHood(), angle,
				Hood.getConstants().tolerance());
	}

	public static Command getSetAngleCommand(double angle) {
		return new AngularPositionCommands.SetAngleHardware(Hood.getHood(), angle);
	}

	public static Command getSettleAngleCommand() {
		return new AngularPositionCommands.SettleAngle(Hood.getHood(), Hood.getConstants().tolerance());
	}
}