// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUsageId;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

/**
 * Class for odometry. Robot code should not use this directly- Instead, use the
 * particular type for
 * your drivetrain (e.g., {@link DifferentialDriveOdometry}). Odometry allows
 * you to track the
 * robot's position on the field over the course of a match using readings from
 * encoders and a
 * gyroscope.
 *
 * <p>
 * Teams can use odometry during the autonomous period for complex tasks like
 * path following.
 * Furthermore, odometry can be used for latency compensation when using
 * computer-vision systems.
 *
 * @param <T> Wheel positions type.
 */
public class Odometry2 {
	private final int m_numModules;

	/**
	 * Constructs a SwerveDriveOdometry object.
	 *
	 * @param kinematics The swerve drive kinematics for your drivetrain.
	 * @param gyroAngle The angle reported by the gyroscope.
	 * @param modulePositions The wheel positions reported by each module.
	 * @param initialPose The starting position of the robot on the field.
	 */
	public Odometry2(
			SwerveDriveKinematics kinematics,
			Rotation2d gyroAngle,
			SwerveModulePosition[] modulePositions,
			Pose2d initialPose) {
		m_kinematics = kinematics;
		m_poseMeters = initialPose;
		m_gyroOffset = gyroAngle.unaryMinus().rotateBy(m_poseMeters.getRotation());
		m_previousAngle = m_poseMeters.getRotation();
		m_previousWheelPositions = m_kinematics.copy(modulePositions);

		m_numModules = modulePositions.length;

		MathSharedStore.reportUsage(MathUsageId.kOdometry_SwerveDrive, 1);
	}

	/**
	 * Constructs a SwerveDriveOdometry object with the default pose at the origin.
	 *
	 * @param kinematics The swerve drive kinematics for your drivetrain.
	 * @param gyroAngle The angle reported by the gyroscope.
	 * @param modulePositions The wheel positions reported by each module.
	 */
	public Odometry2(
			SwerveDriveKinematics kinematics,
			Rotation2d gyroAngle,
			SwerveModulePosition[] modulePositions) {
		this(kinematics, gyroAngle, modulePositions, Pose2d.kZero);
	}

	private final Kinematics<?, SwerveModulePosition[]> m_kinematics;
	private Pose2d m_poseMeters;

	private Rotation2d m_gyroOffset;

	// Always equal to m_poseMeters.getRotation()
	private Rotation2d m_previousAngle;

	private final SwerveModulePosition[] m_previousWheelPositions;

	/**
	 * Resets the robot's position on the field.
	 *
	 * <p>
	 * The gyroscope angle does not need to be reset here on the user's robot code.
	 * The library
	 * automatically takes care of offsetting the gyro angle.
	 *
	 * @param gyroAngle The angle reported by the gyroscope.
	 * @param wheelPositions The current encoder readings.
	 * @param poseMeters The position on the field that your robot is at.
	 */
	public void resetPosition(Rotation2d gyroAngle, SwerveModulePosition[] wheelPositions, Pose2d poseMeters) {
		m_poseMeters = poseMeters;
		m_previousAngle = m_poseMeters.getRotation();
		m_gyroOffset = gyroAngle.unaryMinus().rotateBy(m_poseMeters.getRotation());
		m_kinematics.copyInto(wheelPositions, m_previousWheelPositions);
	}

	/**
	 * Resets the pose.
	 *
	 * @param poseMeters The pose to reset to.
	 */
	public void resetPose(Pose2d poseMeters) {
		m_gyroOffset = m_gyroOffset
				.rotateBy(m_poseMeters.getRotation().unaryMinus())
				.rotateBy(poseMeters.getRotation());
		m_poseMeters = poseMeters;
		m_previousAngle = m_poseMeters.getRotation();
	}

	/**
	 * Resets the translation of the pose.
	 *
	 * @param translation The translation to reset to.
	 */
	public void resetTranslation(Translation2d translation) {
		m_poseMeters = new Pose2d(translation, m_poseMeters.getRotation());
	}

	/**
	 * Resets the rotation of the pose.
	 *
	 * @param rotation The rotation to reset to.
	 */
	public void resetRotation(Rotation2d rotation) {
		m_gyroOffset = m_gyroOffset.rotateBy(m_poseMeters.getRotation().unaryMinus()).rotateBy(rotation);
		m_poseMeters = new Pose2d(m_poseMeters.getTranslation(), rotation);
		m_previousAngle = m_poseMeters.getRotation();
	}

	/**
	 * Returns the position of the robot on the field.
	 *
	 * @return The pose of the robot (x and y are in meters).
	 */
	public Pose2d getPoseMeters() {
		return m_poseMeters;
	}

	/**
	 * Updates the robot's position on the field using forward kinematics and
	 * integration of the pose
	 * over time. This method takes in an angle parameter which is used instead of
	 * the angular rate
	 * that is calculated from forward kinematics, in addition to the current
	 * distance measurement at
	 * each wheel.
	 *
	 * @param gyroAngle The angle reported by the gyroscope.
	 * @param wheelPositions The current encoder readings.
	 * @return The new pose of the robot.
	 */
	public Pair<Pose2d, Translation2d> updateWithVelocity(Rotation2d gyroAngle, SwerveModulePosition[] wheelPositions) {
		var angle = gyroAngle.rotateBy(m_gyroOffset);

		var twist = m_kinematics.toTwist2d(m_previousWheelPositions, wheelPositions);
		twist.dtheta = angle.minus(m_previousAngle).getRadians();

		var newPose = m_poseMeters.exp(twist);

		m_kinematics.copyInto(wheelPositions, m_previousWheelPositions);
		m_previousAngle = angle;
		m_poseMeters = new Pose2d(newPose.getTranslation(), angle);
		return Pair.of(m_poseMeters, new Translation2d(twist.dx / 0.02, twist.dy / 0.02));
	}
}
