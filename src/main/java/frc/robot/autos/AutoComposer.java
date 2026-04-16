package frc.robot.autos;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.NoSuchElementException;
import java.util.Optional;

import choreo.Choreo;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Subsystems.AgitatorConstants;
import frc.robot.Constants.Subsystems.IntakeConstants;
import frc.robot.commands.AimCommands;
import frc.robot.commands.AimCommands.AdjustAim;
import frc.robot.commands.AngularPositionCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.HoodCommands;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.TransportCommands;
import frc.robot.commands.TurretCommands;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

public class AutoComposer {
	private static TimedRobot m_robot;

	public AutoComposer(TimedRobot robot) {
		m_robot = robot;
	}

	public static Command getChildTrajectory(Trajectory<?> trajectory) {
		List<Pose2d> jelly = new ArrayList<>(Arrays.asList(trajectory.getPoses()));
		Pose2d start = jelly.remove(0);
		Pose2d end = jelly.remove(jelly.size() - 1);
		return Commands.parallel(
				Commands.repeatingSequence(new AimCommands.HubLookCommand()),
				Commands.sequence(
						new DriveCommands.ResetOdometry(start),
						new DriveCommands.NinjaStar(start, false),
						Commands.sequence(
								jelly.stream().map(DriveCommands.YOLONinjaStar::new).toArray(Command[]::new)),
						new DriveCommands.NinjaStar(end, true)));
	}

	public Command getDriveTrajectory(String filename, List<Command> jellies) {
		try {
			Trajectory<?> trajectory = Choreo.loadTrajectory(filename).orElseThrow();
			System.out.println(trajectory.getPoses().length);
			List<Command> splits = trajectory.splits().stream().map(trajectory::getSplit).map(Optional::get)
					.map(AutoComposer::getChildTrajectory).toList();
			List<Command> commands = new ArrayList<>();
			System.out.println(trajectory.splits());
			System.out.printf("%d; %d%n", splits.size(), jellies.size());
			for (int i = 0; i < Math.max(splits.size(), jellies.size()); i++) {
				if (jellies.size() > i) {
					commands.add(jellies.get(i));
				}
				if (splits.size() > i) {
					commands.add(splits.get(i));
				}
			}
			return Commands.sequence(commands.toArray(Command[]::new));
		} catch (NoSuchElementException e) {
			return Commands.print(e.toString());
		}

	}

	public Command getSingleTrajectory(String filename) {
		try {
			Trajectory<?> trajectory = Choreo.loadTrajectory(filename).orElseThrow();
			return getChildTrajectory(trajectory);
		} catch (NoSuchElementException e) {
			return Commands.print(e.toString());
		}
	}

	public Command autoAimShoot() {
		return Commands.sequence(
				new RepeatCommand(new AimCommands.HubAimCommand(
						Vision.getVision().getMidpointEstimator())).withTimeout(2.5), // 1.5
				TransportCommands.getTimedShoot(10)); // 5
	}

	public Command getCenterAuto() {
		return Commands.sequence(
				new DriveCommands.DrivePowerAndTime(0.2, 0, 0, 1),
				autoAimShoot());
	}

	public Command getJamPreventativeShoot() {
		return Commands.parallel(
				new IntakeCommands.Spintake(IntakeConstants.kWheelPower),
				Commands.sequence(
						Commands.parallel(
								Commands.repeatingSequence(
										new TransportCommands.RunAgitatorAtPower(
												AgitatorConstants.kTeleopPower)
														.withTimeout(1.5),
										new TransportCommands.RunAgitatorAtPower(
												-AgitatorConstants.kTeleopPower)
														.withTimeout(.25)),
								Commands.repeatingSequence(
										IntakeCommands.getRunArmAtPowerCommand(-1).withTimeout(1.75 / 2.0),
										IntakeCommands.getRunArmAtPowerCommand(1)
												.withTimeout(1.75 / 2.0)))));
	}

	public Command getShootCommand(double turretAngle, double distance, double shootTime) {
		return Commands.sequence(
				AimCommands.getSetAimCommand(distance),
				TurretCommands.getTurnToAngleSoftwareCommand(turretAngle),
				// TurretCommands.getSettleAngleCommand(),
				AimCommands.getSettleAimCommand(),
				TransportCommands.getTimedShoot(shootTime)).withName("Shoot command");
	}

	public Command getPreloadNeutralOutpostAuto() {
		return Commands.parallel(
				IntakeCommands.getArmOutCombinedCommand(),
				Commands.sequence(
						autoAimShoot(),
						HoodCommands.getHoodDownCommand(),
						getSingleTrajectory("sweep_outpost").withTimeout(14),
						autoAimShoot(),
						getSingleTrajectory("sweep_outpost2").withTimeout(3),
						autoAimShoot()));
	}

	public Command getLeftOneShootAuto() {
		return Commands.sequence(
				getShootCommand(92 + (92 - 36), 11.6, 6),
				HoodCommands.getHoodDownCommand()).withName("Left one shoot auto");
	}

	public Command getManualRightShootAuto() {
		return Commands.sequence(
				new AdjustAim(true, 11.6, m_robot).withTimeout(.5),
				Commands.waitSeconds(3), // test changing to 2
				TransportCommands.getTimedShoot(6),
				HoodCommands.getHoodDownCommand()).withName("Right one shoot auto");
	}

	public Command getRightOneShootAuto() {
		return Commands.sequence(
				getShootCommand(45, 11.6, 6)
		// ,
		// new AngularPositionCommands.RunToAngleHardware(Hood.getHood(), 100,
		// Hood.getConstants())
		).withName("Right one shoot auto");
	}

	public Command getRightTwoShootAuto() {
		return Commands.sequence(
				getShootCommand(36, 11.6, 6),
				HoodCommands.getHoodDownCommand(),
				new DriveCommands.DrivePowerAndTime(.2, 0, 0, 3.8),
				new DriveCommands.DrivePowerAndTime(0, 0.2, 0, 1),
				new DriveCommands.DrivePowerAndTime(-0.2, 0, 0, 0.25), // Potentially remove to save time

				// addition
				IntakeCommands.getArmOutCombinedCommand(),
				// addition

				getShootCommand(65 + 2, 18, 20)).withName("Right two shoot auto");
	}

	public Command getRightTwoShootAutoWithVision() {
		return new SequentialCommandGroup(
				new AimCommands.HubAimCommand(Vision.getVision().getFieldPoseEstimator()),
				AimCommands.getSettleAimCommand(),
				new ParallelRaceGroup(
						new RepeatCommand(new AimCommands.HubAimCommand(Vision.getVision().getFieldPoseEstimator())),
						TransportCommands.getTimedShoot(3)),
				// getShootCommand(36, 11.6, 6),
				HoodCommands.getHoodDownCommand(),
				new DriveCommands.VisionDriveDistance(Units.feetToMeters(10.25), 0, 0),
				// IntakeCommands.getArmOutCombinedCommand(),
				new AimCommands.HubAimCommand(Vision.getVision().getFieldPoseEstimator()),
				AimCommands.getSettleAimCommand(),
				new ParallelRaceGroup(
						new RepeatCommand(new AimCommands.HubAimCommand(Vision.getVision().getFieldPoseEstimator())),
						TransportCommands.getTimedShoot(3)));
	}

	public Command getRightTwoShootAutoBump() {
		return Commands.sequence(
				getShootCommand(36, 9.1, 6),
				HoodCommands.getHoodDownCommand(),
				new DriveCommands.DrivePowerAndTime(.2, 0, 0, 3.8),
				new DriveCommands.DrivePowerAndTime(0, 0.2, 0, 1.5),

				// addition
				IntakeCommands.getArmOutCombinedCommand(),
				// addition

				getShootCommand(65 + 2, 18, 20)).withName("Right two shoot auto front of bump");
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
												new AimCommands.HubAimCommand(
														Vision.getVision().getFieldPoseEstimator())))),
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
				getShootCommand(36, 11.6, 6),
				HoodCommands.getHoodDownCommand(),
				new DriveCommands.DrivePowerAndTime(.2, 0, 0, 3.8),
				new DriveCommands.DrivePowerAndTime(0, 0.2, 0, 1),
				new DriveCommands.DrivePowerAndTime(-.2, 0, 0, .5),

				// addition
				Commands.parallel(
						IntakeCommands.getArmOutCombinedCommand(),
						new IntakeCommands.Spintake(1)),
				// addition

				getShootCommand(65 + 2, 18, 20)).withName("Right two shoot auto");
	}

}
