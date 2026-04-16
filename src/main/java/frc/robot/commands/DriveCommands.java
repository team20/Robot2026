package frc.robot.commands;

import static frc.robot.Constants.DriveConstants.*;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AngleUtility;
import frc.robot.ControlUtils.ClampedP;
import frc.robot.ControlUtils.ScaledJoystick;
import frc.robot.Filter.RejectionFilter;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Vision;

public class DriveCommands {

	public enum Tolerances {
		COARSE(0.15, 10),
		FINE_TRANSLATION(0.05, 10),
		FINE_ROTATION(0.15, 2.5),
		FINEST(0.05, 2.5);

		private final double m_translation;
		private final double m_rotation;

		private Tolerances(double translation, double rotation) {
			m_translation = translation;
			m_rotation = rotation;
		}

		public double getTranslation() {
			return m_translation;
		}

		public double getRotation() {
			return m_rotation;
		}
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
	public static class JoystickDrive extends Command {
		private final ScaledJoystick m_joystick;
		private final DoubleSupplier m_rotation;
		private final BooleanSupplier m_isRobotRelative;

		public JoystickDrive(DoubleSupplier forwardSpeed, DoubleSupplier strafeSpeed,
				DoubleSupplier rotation, BooleanSupplier isRobotRelative) {
			m_joystick = new ScaledJoystick(forwardSpeed, strafeSpeed, kDeadzone);
			m_rotation = rotation;
			m_isRobotRelative = isRobotRelative;
			setName("Drive With Joysticks");
			addRequirements(Drive.getDrive());
		}

		// Called every time the scheduler runs while the command is scheduled.
		@Override
		public void execute() {
			double rotationStick = MathUtil.applyDeadband(m_rotation.getAsDouble(), kDeadzone);
			Drive.drive(m_joystick.getX(), m_joystick.getY(), rotationStick, m_isRobotRelative.getAsBoolean());
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

	public static class DrivePowerAndTime extends Command {
		private final double m_fwdPower;
		private final double m_strafePower;
		private final double m_rotSpeed;
		private final double m_time;
		private Timer m_timer = new Timer();

		public DrivePowerAndTime(double fwdPower, double strafePower, double rot, double time) {
			m_fwdPower = fwdPower;
			m_strafePower = strafePower;
			m_rotSpeed = rot;
			m_time = time;
			setName("Drive For Power and Time");
			addRequirements(Drive.getDrive());
		}

		@Override
		public void initialize() {
			m_timer.reset();
			m_timer.start();
		}

		@Override
		public void execute() {
			Drive.drive(m_fwdPower, m_strafePower, m_rotSpeed, true);
		}

		// Called once the command ends or is interrupted.
		@Override
		public void end(boolean interrupted) {
			m_timer.stop();
			Drive.drive(0, 0, 0, true);
		}

		// Returns true when the command should end.
		@Override
		public boolean isFinished() {
			return m_time > 0 && m_timer.hasElapsed(m_time);
		}

	}

	public static class VisionDriveDistance extends Command {
		private final double m_distanceX;
		private final double m_distanceY;
		private final double m_angle;
		private Pose2d m_targetPose;
		private static final double m_tolerance = .05; // When to stop trying!!
		private static final double m_thetaTolerance = 2;

		public VisionDriveDistance(double distanceX, double distanceY, double angle) {
			m_distanceX = distanceX;
			m_distanceY = distanceY;
			m_angle = angle;
			setName("Drive For A Distance");
			addRequirements(Drive.getDrive());
		}

		@Override
		public void initialize() {
			Pose2d current = getCurrentPosition();

			m_targetPose = new Pose2d(current.getX() + m_distanceX, current.getY() + m_distanceY,
					Rotation2d.fromDegrees(current.getRotation().getDegrees() + m_angle));
		}

		public Pose2d getCurrentPosition() {
			Pose2d visionPose = Vision.getVision().getFieldPoseEstimator().getPose();
			Pose2d odometryPose = Drive.getPose();
			return new Pose2d(visionPose.getX(), visionPose.getY(), odometryPose.getRotation());
		}

		public double[] getError() {
			Pose2d currentPos = getCurrentPosition();
			return new double[] {
					currentPos.getX() - m_targetPose.getX(), // X error
					currentPos.getY() - m_targetPose.getY(), // Y error
					currentPos.getRotation().getDegrees() - m_targetPose.getRotation().getDegrees() }; // Theta error
		}

		@Override
		public void execute() {
			double speedX, speedY, rotation;
			{
				double error = getError()[0];
				double minPower = .05;
				double maxPower = .3;
				double maxErr = 1; // When to start slowing down!!
				speedX = -ClampedP.clampedP(
						error, minPower, maxPower, maxErr,
						m_tolerance);
			}
			{
				double errorY = getError()[1];
				double minPower = .05;
				double maxPower = .1;
				double maxErr = .5; // When to start slowing down!!
				speedY = -ClampedP.clampedP(errorY, minPower, maxPower, maxErr, m_tolerance);
			}

			{
				double errorTheta = getError()[2];
				double minPower = .1;
				double maxPower = .3;
				double maxErr = 20; // When to start slowing down!!
				rotation = ClampedP.clampedP(errorTheta, minPower, maxPower, maxErr, m_thetaTolerance);
			}

			SmartDashboard.putNumber("Drive Command/X Error", getError()[0]);
			SmartDashboard.putNumber("Drive Command/Y Error", getError()[1]);
			SmartDashboard.putNumber("Drive Command/Theta Error", getError()[2]);

			// rotation = .03;
			Drive.drive(speedX, speedY, rotation, false);
		}

		// Called once the command ends or is interrupted.
		@Override
		public void end(boolean interrupted) {
			Drive.stop();
		}

		// Returns true when the command should end.
		@Override
		public boolean isFinished() {
			// return (Math.abs(distanceY - Math.abs(m_distanceY)) < m_tolerance);
			return (Math.abs(getError()[0]) < m_tolerance) && (Math.abs(getError()[1]) < m_tolerance) &&
					(Math.abs(getError()[2]) < m_thetaTolerance);
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

	public static class CombinedNinjaStar extends Command {
		private static final double kMaxSpeedMetersPerSecond = 4.5;
		private static final double kLookAhead = 0.5;
		private static final double kFilter = 0.05;
		private final Pose2d[] m_poses;
		private final double m_maxTranslationSpeed;
		private final double m_minTranslationSpeed;
		private final double m_acceleration;
		private final double m_rotationSpeed;
		private final Tolerances m_tolerance;
		private double m_currentSpeed;
		private double m_currentSpeedX;
		private double m_currentSpeedY;
		private final static StructPublisher<Pose2d> m_posePublisher = NetworkTableInstance.getDefault()
				.getStructTopic("/SmartDashboard/Desired Pose YOLO", Pose2d.struct).publish();
		private final RejectionFilter m_filter = new RejectionFilter(kFilter, kFilter);

		public CombinedNinjaStar(Pose2d[] poses, double maxTranslationSpeed, double minTranslationSpeed,
				double rampTime, double rotationSpeed, Tolerances tolerance) {
			m_poses = poses;
			m_minTranslationSpeed = minTranslationSpeed;
			m_maxTranslationSpeed = maxTranslationSpeed;
			m_rotationSpeed = rotationSpeed;
			m_tolerance = tolerance;
			m_acceleration = (m_maxTranslationSpeed - m_minTranslationSpeed) / rampTime;
			setName("Drive To A Pose");
			addRequirements(Drive.getDrive());
		}

		public CombinedNinjaStar(Pose2d[] poses, double maxTranslationSpeed, double minTranslationSpeed,
				double rampTime, double rotationSpeed) {
			this(poses, maxTranslationSpeed, minTranslationSpeed, rampTime, rotationSpeed, Tolerances.FINE_TRANSLATION);
		}

		public CombinedNinjaStar(Pose2d[] poses) {
			this(poses, 0.5, 0.05, 5, 1);
		}

		private double computeTimeLeft(double distance) {
			double maxMetersPerSecond = kMaxSpeedMetersPerSecond * m_maxTranslationSpeed;
			double guess = 2 * distance / maxMetersPerSecond;
			for (int i = 0; i < 10; i++) {
				double derivative = maxMetersPerSecond
						* Math.pow(Math.sin(m_acceleration * guess / m_maxTranslationSpeed), 2);
				double value = maxMetersPerSecond * (guess / 2 - m_maxTranslationSpeed / (4 * m_acceleration)
						* Math.sin(2 * m_acceleration * guess / m_maxTranslationSpeed)) - distance;
				guess -= value / derivative;
			}
			return guess;
		}

		private double determineTargetIndex(Pose2d current) {
			double max = 0;
			double target = -1;
			for (int i = 1; i < m_poses.length; i++) {
				Translation2d trajectory = m_poses[i].getTranslation().minus(m_poses[i - 1].getTranslation());
				Translation2d first = current.getTranslation().minus(m_poses[i - 1].getTranslation());
				Translation2d last = m_poses[i].getTranslation().minus(current.getTranslation());
				double score = Math.exp(1 - (first.getNorm() + last.getNorm()) / trajectory.getNorm());
				if (score > max) {
					Translation2d parallel = first.rotateBy(trajectory.getAngle().unaryMinus());
					max = score;
					target = i - 1 + parallel.getX() / trajectory.getNorm();
				}
			}
			return target;
		}

		private double calculateNewSpeed(Pose2d current) {
			Pose2d end = m_poses[m_poses.length - 1];
			double distance = current.getTranslation().getDistance(end.getTranslation());
			double timeLeft = computeTimeLeft(distance);
			double maxSpeed = m_maxTranslationSpeed
					* Math.pow(Math.sin(m_acceleration * timeLeft / m_maxTranslationSpeed), 2);
			double acceleration = m_acceleration
					* Math.sqrt(Math.max(1 - Math.pow(2 * m_currentSpeed / m_maxTranslationSpeed - 1, 2), 0));
			return Math.min(m_currentSpeed + acceleration * 0.02, maxSpeed);
		}

		@Override
		public void initialize() {
			m_currentSpeed = m_minTranslationSpeed;
		}

		@Override
		public void execute() {
			Pose2d current = Drive.getPose();
			m_currentSpeed = calculateNewSpeed(current);
			double trueTarget = Math.min(m_filter.calculate(determineTargetIndex(current)), 0);
			int nextTarget = (int) trueTarget + 1;
			Pose2d target = m_poses[nextTarget - 1]
					.interpolate(m_poses[nextTarget], (kLookAhead - 1) * (nextTarget - trueTarget) + 1);
			m_posePublisher.accept(target);
			double errorX = current.getX() - target.getX();
			double errorY = current.getY() - target.getY();
			double errorHypot = Math.hypot(errorX, errorY);
			if (errorHypot > 0.01) {
				m_currentSpeedX = m_currentSpeed * errorX / errorHypot;
				m_currentSpeedY = m_currentSpeed * errorY / errorHypot;
			}
			SmartDashboard.putNumber("CombinedNinjaStar/X Speed", m_currentSpeedX);
			SmartDashboard.putNumber("CombinedNinjaStar/Y Speed", m_currentSpeedY);
			double errorTheta = current.getRotation().getDegrees() - target.getRotation().getDegrees();
			SmartDashboard.putNumber("CombinedNinjaStar/X Error", errorX);
			SmartDashboard.putNumber("CombinedNinjaStar/Y Error", errorY);
			SmartDashboard.putNumber("CombinedNinjaStar/θ Error", errorTheta);
			double speedTheta = ClampedP.clampedP(errorTheta, 0.01, m_rotationSpeed, 45, 0);
			Drive.drive(m_currentSpeedX, m_currentSpeedY, speedTheta, false);
		}

		// Returns true when the command should end.
		@Override
		public boolean isFinished() {
			Pose2d end = m_poses[m_poses.length - 1];
			Pose2d current = Drive.getPose();
			double errorX = Math.abs(end.getX() - current.getX());
			double errorY = Math.abs(end.getY() - current.getY());
			double errorTheta = AngleUtility
					.minDifference(current.getRotation().getDegrees(), end.getRotation().getDegrees());
			return Math.max(errorX, errorY) < m_tolerance.getTranslation() && errorTheta < m_tolerance.getRotation();
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