package frc.robot.autos;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AimCommands.AdjustAim;
import frc.robot.commands.AngularPositionCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.TransportCommands;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Turret;

public class AutoComposer {
	private static TimedRobot m_robot;

	public AutoComposer(TimedRobot robot) {
		m_robot = robot;
	}

	private Command getShootCommand(double turretAngle, double distance, double shootTime) {
		return new SequentialCommandGroup(
				new AngularPositionCommands.RunToAngleHardware(Turret.getTurret(), turretAngle,
						Turret.getConstants()).withTimeout(.5),
				new AdjustAim(true, distance, m_robot).withTimeout(.5),
				new WaitCommand(3),
				TransportCommands.getTimedShoot(shootTime));
	}

	public Command getRedRightTwoShootAuto() {
		return new SequentialCommandGroup(
				getShootCommand(36, 11.6, 6),
				new AngularPositionCommands.RunToAngleHardware(Hood.getHood(), 100,
						Hood.getConstants()),
				new DriveCommands.DrivePowerAndTime(.2, 0, 0, 3.8),
				new DriveCommands.DrivePowerAndTime(0, 0.2, 0, 1),
				getShootCommand(65, 18, 20));
	}

}
