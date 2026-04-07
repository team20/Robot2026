package frc.robot.autos;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Subsystems.AgitatorConstants;
import frc.robot.commands.AimCommands;
import frc.robot.commands.AimCommands.AdjustAim;
import frc.robot.commands.AngularPositionCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.HoodCommands;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.TransportCommands;
import frc.robot.commands.TurretCommands;
import frc.robot.subsystems.Turret;

public class AutoComposer {
	private static TimedRobot m_robot;

	public AutoComposer(TimedRobot robot) {
		m_robot = robot;
	}

	public Command getShootCommand(double turretAngle, Distance distance, double shootTime) {
		return Commands.sequence(
				AimCommands.getSetAimCommand(distance),
				TurretCommands.getTurnToAngleSoftwareCommand(turretAngle),
				// TurretCommands.getSettleAngleCommand(),
				AimCommands.getSettleAimCommand(),
				TransportCommands.getTimedShoot(shootTime)).withName("Shoot command");
	}

	public Command getLeftOneShootAuto() {
		return Commands.sequence(
				getShootCommand(92 + (92 - 36), Feet.of(11.6), 6),
				HoodCommands.getHoodDownCommand()).withName("Left one shoot auto");
	}

	public Command getManualRightShootAuto() {
		return Commands.sequence(
				new AdjustAim(true, Feet.of(11.6), m_robot).withTimeout(.5),
				Commands.waitSeconds(3), // test changing to 2
				TransportCommands.getTimedShoot(6),
				HoodCommands.getHoodDownCommand()).withName("Right one shoot auto");
	}

	public Command getRightOneShootAuto() {
		return Commands.sequence(
				getShootCommand(45, Feet.of(11.6), 6)
		// ,
		// new AngularPositionCommands.RunToAngleHardware(Hood.getHood(), 100,
		// Hood.getConstants())
		).withName("Right one shoot auto");
	}

	public Command getRightTwoShootAuto() {
		return Commands.sequence(
				getShootCommand(36, Feet.of(11.6), 6),
				HoodCommands.getHoodDownCommand(),
				new DriveCommands.DrivePowerAndTime(.2, 0, 0, 3.8),
				new DriveCommands.DrivePowerAndTime(0, 0.2, 0, 1),
				new DriveCommands.DrivePowerAndTime(-0.2, 0, 0, 0.25), // Potentially remove to save time

				// addition
				IntakeCommands.getArmOutCombinedCommand(),
				// addition

				getShootCommand(65 + 2, Feet.of(18), 20)).withName("Right two shoot auto");
	}

	public Command getRightTwoShootAutoWithVision() {
		return new SequentialCommandGroup(
				new AimCommands.HubAimCommand(),
				AimCommands.getSettleAimCommand(),
				new ParallelRaceGroup(
						new RepeatCommand(new AimCommands.HubAimCommand()),
						TransportCommands.getTimedShoot(3)),
				// getShootCommand(36, 11.6, 6),
				HoodCommands.getHoodDownCommand(),
				new DriveCommands.VisionDriveDistance(Units.feetToMeters(10.25), 0, 0),
				// IntakeCommands.getArmOutCombinedCommand(),
				new AimCommands.HubAimCommand(),
				AimCommands.getSettleAimCommand(),
				new ParallelRaceGroup(
						new RepeatCommand(new AimCommands.HubAimCommand()),
						TransportCommands.getTimedShoot(3)));
	}

	public Command getRightTwoShootAutoBump() {
		return Commands.sequence(
				getShootCommand(36, Feet.of(9.1), 6),
				HoodCommands.getHoodDownCommand(),
				new DriveCommands.DrivePowerAndTime(.2, 0, 0, 3.8),
				new DriveCommands.DrivePowerAndTime(0, 0.2, 0, 1.5),

				// addition
				IntakeCommands.getArmOutCombinedCommand(),
				// addition

				getShootCommand(65 + 2, Feet.of(18), 20)).withName("Right two shoot auto front of bump");
	}

	public Command getVelocity1Testcommand() {
		return new DriveCommands.DrivePowerAndTime(-0.3, 0, 0, 5);
	}

	public Command getVelocityTestCommand() {
		return Commands.sequence(
				new DriveCommands.DrivePowerAndTime(.2, 0, 0, 1),
				Commands.parallel(
						Commands.sequence(
								new AngularPositionCommands.SetAngleHardware(Turret.getTurret(), 135),
								new AngularPositionCommands.SettleAngle(Turret.getTurret(), 10),
								Commands.race(
										new TransportCommands.RunAgitatorAtPower(AgitatorConstants.kTeleopPower),
										Commands.repeatingSequence(
												new AimCommands.HubAimCommand()))),
						Commands.sequence(
								new DriveCommands.DrivePowerAndTime(.07, .07, 0, 8))));
	}

	public Command getVelocityTestCommandReverse() {
		return Commands.sequence(
				HoodCommands.getHoodDownCommand(),
				new DriveCommands.DrivePowerAndTime(-.3, -.3, 0, 1.2),
				new DriveCommands.DrivePowerAndTime(-.2, 0, 0, 1));
	}

	public Command getRightTwoShootAutoWithIntake() {
		return Commands.sequence(
				getShootCommand(36, Feet.of(11.6), 6),
				HoodCommands.getHoodDownCommand(),
				new DriveCommands.DrivePowerAndTime(.2, 0, 0, 3.8),
				new DriveCommands.DrivePowerAndTime(0, 0.2, 0, 1),
				new DriveCommands.DrivePowerAndTime(-.2, 0, 0, .5),

				// addition
				Commands.parallel(
						IntakeCommands.getArmOutCombinedCommand(),
						new IntakeCommands.Spintake(1)),
				// addition

				getShootCommand(65 + 2, Feet.of(18), 20)).withName("Right two shoot auto");
	}

}
