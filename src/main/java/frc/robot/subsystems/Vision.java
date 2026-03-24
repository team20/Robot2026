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
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Aim;
import frc.robot.Constants.Subsystems.VisionConstants;
import frc.robot.PoseUtils;
import frc.robot.SensorFusion;

public class Vision extends SubsystemBase {
	private static Vision s_vision;
	private final PhotonCamera m_camera = new PhotonCamera("TurretCamera");
	private final InternalEstimator m_hubEstimator = new FieldPoseEstimator();
	private final InternalEstimator m_midpointEstimator = new MidpointEstimator();
	private final FusedEstimator m_fusedEstimator = new FusedEstimator();

	public interface AngleDistanceEstimator {
		public double getAngle();

		public Distance getDistance();
	}

	private interface InternalEstimator extends AngleDistanceEstimator {
		public void update(List<PhotonTrackedTarget> targets);
	}

	public class FieldPoseEstimator implements InternalEstimator {
		private double m_distance;
		private double m_angle;
		private final StructPublisher<Pose2d> m_estimatedPoseTopic = NetworkTableInstance.getDefault()
				.getStructTopic("SmartDashboard/Vision/FieldPose/Estimated Camera Pose", Pose2d.struct)
				.publish();

		public void update(List<PhotonTrackedTarget> targets) {
			PoseUtils.estimateCamPoseStdDev(targets).ifPresent(pose -> {
				Translation2d difference = PoseUtils.getHub().minus(pose.pose()).getTranslation();
				m_angle = -difference.getAngle().getDegrees();
				m_distance = difference.getNorm();

				m_estimatedPoseTopic.accept(pose.pose());
				SmartDashboard.putNumber("Vision/FieldPose/Angle To Hub", m_angle);
				SmartDashboard.putNumber("Vision/FieldPose/Distance To Hub", Units.metersToFeet(m_distance));
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
	}

	public class MidpointEstimator implements InternalEstimator {
		private double m_distance;
		private double m_angle;

		public void update(List<PhotonTrackedTarget> targets) {
			double minAngle = 0;
			double maxAngle = 0;
			double minDistance = 0;
			double maxDistance = 0;
			for (PhotonTrackedTarget target : targets) {
				if (!VisionConstants.kTrackableTags.contains(target.getFiducialId()))
					continue;
				double distance = target.getBestCameraToTarget().getX();
				SmartDashboard.putNumber("Tag Dist/" + target.getFiducialId(), distance);
				if (minAngle == 0 && maxAngle == 0 && minDistance == 0 && maxDistance == 0) {
					minAngle = maxAngle = target.getYaw();
					minDistance = maxDistance = distance;
				} else {
					minAngle = Math.min(target.getYaw(), minAngle);
					maxAngle = Math.max(target.getYaw(), maxAngle);
					minDistance = Math.min(distance, minDistance);
					maxDistance = Math.max(distance, maxDistance);
				}
			}
			// Get midpoint distance + offset to center of hub
			m_distance = Units.metersToFeet((minDistance + maxDistance) / 2) + 23.5 / 12;
			m_angle = (minAngle + maxAngle) / 2;

			SmartDashboard.putNumber("Vision/Midpoint/Angle to Hub", m_angle);
			SmartDashboard.putNumber("Vision/Midpoint/Distance to Hub", m_distance);
		}

		public double getAngle() {
			return m_angle;
		}

		public Distance getDistance() {
			return Feet.of(m_distance);
		}
	}

	public class FusedEstimator implements AngleDistanceEstimator {
		private final SensorFusion m_angleFusion = new SensorFusion(30, 0.02);
		private final SensorFusion m_distanceFusion = new SensorFusion(30, 0.02);
		private double m_distance;
		private double m_angle;
		private Aim m_aim = new Aim.AirtimeRegression();

		public void update(AngleDistanceEstimator accurate, AngleDistanceEstimator consistent) {
			m_angleFusion.update(accurate.getAngle(), consistent.getAngle());
			m_distanceFusion.update(accurate.getDistance().in(Feet), consistent.getDistance().in(Feet));
			m_distance = m_distanceFusion.getValue();
			double airtime = m_aim.getShotAirtime(m_distance);
			double angle = Units.degreesToRadians(m_angleFusion.getValue());
			double angularVelocity = Units.degreesToRadians(m_angleFusion.getDerivative());
			double x = Math.sin(angle) * m_distance;
			double y = Math.cos(angle) * m_distance;
			x -= angularVelocity * m_distance * airtime;
			y -= m_distanceFusion.getDerivative() * airtime;
			m_angle = Units.radiansToDegrees(Math.atan2(x, y));
			SmartDashboard.putNumber("Vision/Fused/Estimated Airtime", airtime);
			SmartDashboard.putNumber("Vision/Fused/Angle to Hub", m_angle);
			SmartDashboard.putNumber("Vision/Fused/Distance to Hub", m_distance);
			SmartDashboard.putNumber("Vision/Fused/Angle Derivative", m_angleFusion.getDerivative());
			SmartDashboard.putNumber("Vision/Fused/Distance Derivative", m_distanceFusion.getDerivative());
		}

		public double getAngle() {
			return m_angle;
		}

		public Distance getDistance() {
			return Feet.of(m_distance);
		}
	}

	public Vision() {
		if (s_vision == null) {
			s_vision = this;
		} else {
			throw new Error("Vision already instantiated");
		}
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
		m_hubEstimator.update(targets);
		m_midpointEstimator.update(targets);
		m_fusedEstimator.update(m_hubEstimator, m_midpointEstimator);
	}

	public AngleDistanceEstimator getMidpointEstimator() {
		return m_midpointEstimator;
	}

	public AngleDistanceEstimator getFieldPoseEstimator() {
		return m_hubEstimator;
	}

	public AngleDistanceEstimator getFusedEstimator() {
		return m_fusedEstimator;
	}
}
