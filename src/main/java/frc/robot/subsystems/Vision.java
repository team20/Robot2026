package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Subsystems.VisionConstants;
import frc.robot.PoseUtils;

public class Vision extends SubsystemBase {
	private static Vision s_vision;
	private final PhotonCamera m_camera;
	private final FieldPoseEstimator m_hubEstimator;
	private final MidpointEstimator m_midpointEstimator;

	public interface AngleDistanceEstimator {
		public double getAngle();

		public Distance getDistance();
	}

	public class FieldPoseEstimator implements AngleDistanceEstimator {
		private double m_distance;
		private double m_angle;
		private double m_angleDerivative;
		private double m_distantDerivative;
		private final StructPublisher<Pose2d> m_estimatedPoseTopic = NetworkTableInstance.getDefault()
				.getStructTopic("SmartDashboard/Vision/FieldPose/Estimated Camera Pose", Pose2d.struct)
				.publish();

		public void updateMidpoint(List<PhotonTrackedTarget> targets) {
			PoseUtils.estimateCamPoseStdDev(targets).ifPresent(pose -> {
				Translation2d difference = PoseUtils.getHub().minus(pose.pose()).getTranslation();
				double angle = -difference.getAngle().getDegrees();
				m_angleDerivative = (angle - m_angle) / 0.02;
				m_angle = angle;
				double distance = difference.getNorm();
				m_distantDerivative = (distance - m_distance) / 0.02;
				m_distance = distance;
				m_estimatedPoseTopic.accept(pose.pose());
				SmartDashboard.putNumber("Vision/FieldPose/Angle To Hub", m_angle);
				SmartDashboard.putNumber("Vision/FieldPose/Distance To Hub", m_distance);
				SmartDashboard.putNumber("Vision/FieldPose/Estimate Pose Deviation/x", pose.xStdDev());
				SmartDashboard.putNumber("Vision/FieldPose/Estimate Pose Deviation/y", pose.yStdDev());
				SmartDashboard.putNumber(
						"Vision/FieldPose/Estimate Pose Deviation/theta",
						pose.thetaStdDev());
			});
		}

		public double getAngle() {
			return m_angle;
		}

		public Distance getDistance() {
			return Meters.of(m_distance);
		}

		public LinearVelocity getDistanceDerivative() {
			return MetersPerSecond.of(m_distantDerivative);
		}

		public AngularVelocity getAngleDerivative() {
			return DegreesPerSecond.of(m_angleDerivative);
		}
	}

	public class MidpointEstimator implements AngleDistanceEstimator {
		private double m_midpointAngle = 0;
		private double m_midpointDistance = 0;

		public void updateMidpoint(List<PhotonTrackedTarget> targets) {
			double minAngle = 0;
			double maxAngle = 0;
			double minDistance = 0;
			double maxDistance = 0;
			for (PhotonTrackedTarget target : targets) {
				if (!VisionConstants.kTrackableTags.contains(target.fiducialId))
					continue;
				double distance = target.getBestCameraToTarget().getX();
				SmartDashboard.putNumber("Tag Dist/" + target.getFiducialId(), distance);
				if (minAngle == 0 && maxAngle == 0 && minDistance == 0 && maxDistance == 0) {
					minAngle = maxAngle = target.yaw;
					minDistance = maxDistance = distance;
				} else {
					minAngle = Math.min(target.yaw, minAngle);
					maxAngle = Math.max(target.yaw, maxAngle);
					minDistance = Math.min(distance, minDistance);
					maxDistance = Math.max(distance, maxDistance);
				}
			}
			// Get midpoint distance + offset to center of hub
			m_midpointDistance = (minDistance + maxDistance) / 2 + Units.inchesToMeters(23.5);
			m_midpointAngle = (minAngle + maxAngle) / 2;

			SmartDashboard.putNumber("Vision/Midpoint/Angle to Hub", m_midpointAngle);

			SmartDashboard.putNumber("Vision/Midpoint/Distance to Hub", m_midpointDistance);
		}

		public double getAngle() {
			return m_midpointAngle;
		}

		public Distance getDistance() {
			return Meters.of(m_midpointDistance);
		}
	}

	public Vision() {
		m_hubEstimator = new FieldPoseEstimator();
		m_midpointEstimator = new MidpointEstimator();
		if (s_vision == null) {
			s_vision = this;
		} else {
			throw new Error("Vision already instantiated");
		}

		m_camera = new PhotonCamera("TurretCamera");
	}

	public static Vision getVision() {
		return s_vision;
	}

	public void periodic() {
		List<PhotonPipelineResult> results = m_camera.getAllUnreadResults();
		if (results.isEmpty()) {
			return;
		}
		PhotonPipelineResult latest = results.get(results.size() - 1);
		List<PhotonTrackedTarget> targets = latest.getTargets();
		m_hubEstimator.updateMidpoint(targets);
		m_midpointEstimator.updateMidpoint(targets);
		SmartDashboard.putNumber("Angle Derivative", m_hubEstimator.getAngleDerivative().in(DegreesPerSecond));
		SmartDashboard.putNumber("Distance Derivative", m_hubEstimator.getDistanceDerivative().in(FeetPerSecond));
	}

	public AngleDistanceEstimator getMidpointEstimator() {
		return m_midpointEstimator;
	}

	public AngleDistanceEstimator getFieldPoseEstimator() {
		return m_hubEstimator;
	}
}
