package frc.robot.commands;

import static frc.robot.Constants.DriveConstants.*;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ClampedP;
import frc.robot.subsystems.Drive;

public class DriveCommands {

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
	public static class JoystickDrive extends Command {
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
			setName("Drive With Joysticks");
			addRequirements(Drive.getDrive());
		}

		// Called every time the scheduler runs while the command is scheduled.
		@Override
		public void execute() {
			double forwardStick = MathUtil.applyDeadband(m_forwardSpeed.getAsDouble(), kDeadzone);
			double forwardSpeed = 2 * Math.asin(forwardStick) / Math.PI;
			double strafeStick = MathUtil.applyDeadband(m_strafeSpeed.getAsDouble(), kDeadzone);
			double strafeSpeed = 2 * Math.asin(strafeStick) / Math.PI;
			double rotationStick = MathUtil.applyDeadband(m_rotation.getAsDouble(), kDeadzone);
			Drive.drive(forwardSpeed, strafeSpeed, rotationStick, m_isRobotRelative.getAsBoolean());
		}

		// Called once the command ends or is interrupted.
		@Override
		public void end(boolean interrupted) {
			Drive.stop();
		}

		// Returns true when the command should end.
		@Override
		public boolean isFinished() {
			return false;
		}
	}

	public static class DriveDistance extends Command {
		private final double m_distance;
		private Pose2d m_initialPose;
		private static final double m_tolerance = .05; // When to stop trying!!

		public DriveDistance(double distance) {
			m_distance = distance;
			setName("Drive For A Distance");
			addRequirements(Drive.getDrive());
		}

		@Override
		public void initialize() {
			m_initialPose = Drive.getPose();
		}

		@Override
		public void execute() {
			Transform2d transform = Drive.getPose().minus(m_initialPose);
			double speed, rotation;
			{
				double distance = transform.getTranslation().getNorm();
				double error = (distance - Math.abs(m_distance)) * Math.signum(m_distance);
				double minPower = .05;
				double maxPower = .1;
				double maxErr = .5; // When to start slowing down!!
				speed = ClampedP.clampedP(error, minPower, maxPower, maxErr, m_tolerance);
			}
			{
				double angle = transform.getTranslation().getAngle().getDegrees();
				double error = 0 - angle;
				double minPower = .01;
				double maxPower = .05;
				double maxErr = 2; // When to start slowing down!!
				rotation = ClampedP.clampedP(error, minPower, maxPower, maxErr, m_tolerance);
			}
			// rotation = .03;
			Drive.drive(speed, 0, rotation, true);
		}

		// Called once the command ends or is interrupted.
		@Override
		public void end(boolean interrupted) {
			Drive.stop();
		}

		// Returns true when the command should end.
		@Override
		public boolean isFinished() {
			double distance = Drive.getPose().minus(m_initialPose).getTranslation().getNorm();
			return Math.abs(distance - Math.abs(m_distance)) < m_tolerance;
		}

	}

	public static class SpinToAngle extends Command {
		private final double m_angle;
		private final double m_speed;
		private Pose2d m_initialPose;

		public SpinToAngle(double angle, double speed) {
			m_angle = angle;
			m_speed = speed;
			setName("Spin To An Angle");
			addRequirements(Drive.getDrive());
		}

		@Override
		public void initialize() {
			m_initialPose = Drive.getPose();
		}

		@Override
		public void execute() {
			double angle = Drive.getPose().minus(m_initialPose).getRotation().getDegrees();
			double error = angle - m_angle;
			double speed = ClampedP.clampedP(error, 0.05, m_speed, 45, 5);
			Drive.drive(0, 0, speed, true);
		}

		// Called once the command ends or is interrupted.
		@Override
		public void end(boolean interrupted) {
			Drive.stop();
		}

		// Returns true when the command should end.
		@Override
		public boolean isFinished() {
			double angle = Drive.getPose().minus(m_initialPose).getRotation().getDegrees();
			return Math.abs(angle - m_angle) < 5;
		}
	}

	public static class NinjaStar extends Command {
		private final Pose2d m_pose;
		private final double m_translationSpeed;
		private final double m_rotationSpeed;

		public NinjaStar(Pose2d pose, double translationSpeed, double rotationSpeed) {
			m_pose = pose;
			m_translationSpeed = translationSpeed;
			m_rotationSpeed = rotationSpeed;
			setName("Drive To A Pose");
			addRequirements(Drive.getDrive());
		}

		@Override
		public void execute() {
			Transform2d error = Drive.getPose().minus(m_pose);
			double speedX = ClampedP.clampedP(error.getX(), 0.05, m_translationSpeed, 1, 0.01);
			double speedY = ClampedP.clampedP(error.getY(), 0.05, m_translationSpeed, 1, 0.01);
			double speedTheta = ClampedP.clampedP(error.getRotation().getDegrees(), 0.05, m_rotationSpeed, 45, 5);
			Drive.drive(speedX, speedY, speedTheta, true);
		}

		// Called once the command ends or is interrupted.
		@Override
		public void end(boolean interrupted) {
			Drive.stop();
		}

		// Returns true when the command should end.
		@Override
		public boolean isFinished() {
			Transform2d error = Drive.getPose().minus(m_pose);
			return error.getTranslation().getNorm() < 0.01 && Math.abs(error.getRotation().getDegrees()) < 5;
		}
	}

	public static class ResetHeading extends Command {
		public ResetHeading() {
			setName("Reset Robot Heading");
			addRequirements(Drive.getDrive());
		}

		@Override
		public void initialize() {
			Drive.resetHeading();
		}

		// Returns true when the command should end.
		@Override
		public boolean isFinished() {
			return true;
		}
	}

	public static class ResetOdometry extends Command {
		private final Pose2d m_pose;

		public ResetOdometry(Pose2d pose) {
			m_pose = pose;
			setName("Reset Drive Odometry");
			addRequirements(Drive.getDrive());
		}

		@Override
		public void initialize() {
			Drive.resetOdometry(m_pose);
		}

		// Returns true when the command should end.
		@Override
		public boolean isFinished() {
			return true;
		}
	}

	public static class ToggleCoastMode extends Command {
		public ToggleCoastMode() {
			setName("Toggle Coast Mode");
			addRequirements(Drive.getDrive());
		}

		@Override
		public void initialize() {
			Drive.toggleCoastMode();
		}

		// Returns true when the command should end.
		@Override
		public boolean isFinished() {
			return true;
		}
	}

	public static class SetCoastMode extends Command {
		private final NeutralModeValue m_mode;

		public SetCoastMode(NeutralModeValue mode) {
			m_mode = mode;
			setName("Set Coast Mode");
			addRequirements(Drive.getDrive());
		}

		@Override
		public void execute() {
			Drive.setCoastMode(m_mode);
		}

		// Returns true when the command should end.
		@Override
		public boolean isFinished() {
			return true;
		}
	}

	public static class Stop extends Command {
		public Stop() {
			setName("Stop Driving");
			addRequirements(Drive.getDrive());
		}

		@Override
		public void initialize() {
			Drive.stop();
		}

		// Returns true when the command should end.
		@Override
		public boolean isFinished() {
			return true;
		}
	}
}