package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ClampedP;
import frc.robot.subsystems.DriveSubsystem;

public class DriveCommand {
	private final DriveSubsystem m_driveSubsystem;

	public DriveCommand(DriveSubsystem driveSubsystem) {
		m_driveSubsystem = driveSubsystem;
	}

	/**
	 * Creates a {@code Command} to drive the robot with joystick input.
	 *
	 * @param forwardSpeed Forward speed supplier. Positive values make the robot
	 *        go forward (+X direction).
	 * @param strafeSpeed Strafe speed supplier. Positive values make the robot
	 *        go to the left (+Y direction).
	 * @param rotation Rotation supplier. Positive values make
	 *        the robot rotate left (CCW direction).
	 * @return a {@code ChassisSpeeds} instance to drive the robot with joystick
	 *         input
	 */
	public class JoystickDrive extends Command {
		private final DoubleSupplier m_forwardSpeed;
		private final DoubleSupplier m_strafeSpeed;
		private final DoubleSupplier m_rotation;
		private final BooleanSupplier m_isRobotRelative;

		public JoystickDrive(DoubleSupplier forwardSpeed, DoubleSupplier strafeSpeed,
				DoubleSupplier rotation, BooleanSupplier isRobotRelative) {
			m_forwardSpeed = forwardSpeed;
			m_strafeSpeed = strafeSpeed;
			m_rotation = rotation;
			m_isRobotRelative = isRobotRelative;
			addRequirements(m_driveSubsystem);
		}

		// Called every time the scheduler runs while the command is scheduled.
		@Override
		public void execute() {

			m_driveSubsystem.drive(
					m_forwardSpeed.getAsDouble(), m_strafeSpeed.getAsDouble(),
					m_rotation.getAsDouble(),
					m_isRobotRelative.getAsBoolean());

		}

		// Called once the command ends or is interrupted.
		@Override
		public void end(boolean interrupted) {
			m_driveSubsystem.stopAllModules();
		}

		// Returns true when the command should end.
		@Override
		public boolean isFinished() {
			return false;
		}
	}

	public class DriveDistanceForTime extends Command {
		private final double m_time;
		private final double m_speed;
		private Rotation2d m_rot;
		private Timer m_timer = new Timer();
		private Pose2d m_initialPose;

		public DriveDistanceForTime(double distance, double speed) {
			m_time = (distance * 3.07) / (speed * 12);
			m_speed = speed;
			addRequirements(m_driveSubsystem);
		}

		@Override
		public void initialize() {
			m_timer.reset();
			m_timer.start();
			m_initialPose = m_driveSubsystem.getPose();
			m_rot = m_driveSubsystem.getModulePositions()[0].angle;
		}

		@Override
		public void execute() {
			double s_x = m_speed * m_rot.getCos();
			double s_y = m_speed * m_rot.getSin();
			m_driveSubsystem.drive(s_y, s_x, 0, true);
		}

		// Called once the command ends or is interrupted.
		@Override
		public void end(boolean interrupted) {
			m_timer.stop();
			m_driveSubsystem.stopAllModules();
			SmartDashboard.putNumber("distance traveled", m_driveSubsystem.getPose().minus(m_initialPose).getX());
		}

		// Returns true when the command should end.
		@Override
		public boolean isFinished() {
			return m_timer.get() >= m_time;
		}
	}

	public class DriveDistance extends Command {
		private final double m_distance;
		private Pose2d m_initialPose;
		private final double m_tolerance;

		public DriveDistance(double distance) {
			m_distance = distance;
			m_tolerance = 0.01; // Does not require user to pass in
			addRequirements(m_driveSubsystem);
		}

		@Override
		public void initialize() {
			m_initialPose = m_driveSubsystem.getPose();
		}

		@Override
		public void execute() {
			double error = m_distance - m_driveSubsystem.getPose().minus(m_initialPose).getX();
			double speed = ClampedP.clampedP(error, 0.5, 2.0, m_distance, m_tolerance) * -1;
			m_driveSubsystem.drive(speed, 0, 0, true);
		}

		// Called once the command ends or is interrupted.
		@Override
		public void end(boolean interrupted) {
			SmartDashboard.putNumber(
					"distance traveled", m_driveSubsystem.getPose().minus(m_initialPose).getX());
			SmartDashboard.putNumber(
					"initial pose", m_initialPose.getX());
			m_driveSubsystem.stopAllModules();
		}

		// Returns true when the command should end.
		@Override
		public boolean isFinished() {
			double distance = m_driveSubsystem.getPose().minus(m_initialPose).getX();
			return Math.abs(distance - m_distance) < m_tolerance;
		}

	}

	public class SpinToAngle extends Command {
		private final double m_angle;
		private final double m_speed;
		private Pose2d m_initialPose;

		public SpinToAngle(double angle, double speed) {
			m_angle = angle;
			m_speed = speed;
			addRequirements(m_driveSubsystem);
		}

		@Override
		public void initialize() {
			m_initialPose = m_driveSubsystem.getPose();
		}

		@Override
		public void execute() {
			double angle = m_driveSubsystem.getPose().minus(m_initialPose).getRotation().getDegrees();
			double error = angle - m_angle;
			double speed = ClampedP.clampedP(error, 0.05, m_speed, 45, 5);
			m_driveSubsystem.drive(0, 0, speed, true);
		}

		// Called once the command ends or is interrupted.
		@Override
		public void end(boolean interrupted) {
			m_driveSubsystem.stopAllModules();
		}

		// Returns true when the command should end.
		@Override
		public boolean isFinished() {
			double angle = m_driveSubsystem.getPose().minus(m_initialPose).getRotation().getDegrees();
			return Math.abs(angle - m_angle) < 5;
		}
	}

	public class NinjaStar extends Command {
		private final Pose2d m_pose;
		private final double m_translationSpeed;
		private final double m_rotationSpeed;

		public NinjaStar(Pose2d pose, double translationSpeed, double rotationSpeed) {
			m_pose = pose;
			m_translationSpeed = translationSpeed;
			m_rotationSpeed = rotationSpeed;
			addRequirements(m_driveSubsystem);
		}

		@Override
		public void execute() {
			Transform2d error = m_driveSubsystem.getPose().minus(m_pose);
			double speedX = ClampedP.clampedP(error.getX(), 0.05, m_translationSpeed, 1, 0.01);
			double speedY = ClampedP.clampedP(error.getY(), 0.05, m_translationSpeed, 1, 0.01);
			double speedTheta = ClampedP.clampedP(error.getRotation().getDegrees(), 0.05, m_rotationSpeed, 45, 5);
			m_driveSubsystem.drive(speedX, speedY, speedTheta, false);
		}

		// Called once the command ends or is interrupted.
		@Override
		public void end(boolean interrupted) {
			m_driveSubsystem.stopAllModules();
		}

		// Returns true when the command should end.
		@Override
		public boolean isFinished() {
			Transform2d error = m_driveSubsystem.getPose().minus(m_pose);
			return error.getTranslation().getNorm() < 0.01 && Math.abs(error.getRotation().getDegrees()) < 5;
		}
	}

	public class ResetHeading extends Command {
		@Override
		public void execute() {
			m_driveSubsystem.resetHeading();
			addRequirements(m_driveSubsystem);
		}

		// Returns true when the command should end.
		@Override
		public boolean isFinished() {
			return true;
		}
	}

	public class ResetOdometry extends Command {
		private final Pose2d m_pose;

		public ResetOdometry(Pose2d pose) {
			m_pose = pose;
			addRequirements(m_driveSubsystem);
		}

		@Override
		public void execute() {
			m_driveSubsystem.resetOdometry(m_pose);
		}

		// Returns true when the command should end.
		@Override
		public boolean isFinished() {
			return true;
		}
	}

	public class ToggleCoastMode extends Command {

		@Override
		public void execute() {
			m_driveSubsystem.toggleCoastMode();
			addRequirements(m_driveSubsystem);
		}

		// Returns true when the command should end.
		@Override
		public boolean isFinished() {
			return true;
		}
	}

	public class SetNeutralMode extends Command {
		private final NeutralModeValue m_mode;

		public SetNeutralMode(NeutralModeValue mode) {
			m_mode = mode;
			addRequirements(m_driveSubsystem);
		}

		@Override
		public void execute() {
			m_driveSubsystem.toggleCoastMode();
		}

		// Returns true when the command should end.
		@Override
		public boolean isFinished() {
			return true;
		}
	}

	public class Stop extends Command {
		public Stop() {
			addRequirements(m_driveSubsystem);
		}

		@Override
		public void initialize() {
			m_driveSubsystem.stopAllModules();
		}

		// Returns true when the command should end.
		@Override
		public boolean isFinished() {
			return true;
		}
	}

	public class TurnSteerToAngle extends Command {
		private final double m_angle;
		private final double m_tolerance;

		public TurnSteerToAngle(double angle) {
			m_angle = angle;
			m_tolerance = 3; // Can be constant from command to command
			addRequirements(m_driveSubsystem);
		}

		@Override
		public void execute() {
			m_driveSubsystem.turnSteerToAngle(m_angle);
		}

		// Finishes when all four modules are within angle tolerance
		@Override
		public boolean isFinished() {
			SwerveModulePosition[] poses = m_driveSubsystem.getModulePositions();
			for (int i = 0; i < 4; i++) {
				if (Math.abs(poses[i].angle.getDegrees() - m_angle) > m_tolerance) {
					return false;
				}
			}
			return true;
		}
	}
}
