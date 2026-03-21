package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Subsystems.VisionConstants;

public class Vision extends SubsystemBase {
	private static Vision s_vision;
	private final PhotonCamera m_camera;
	private final MidpointEstimator m_hubEstimator;

	public class MidpointEstimator {
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
		}

		public double getMidpointAngle() {
			return m_midpointAngle;
		}

		public double getMidpointDistance() {
			return m_midpointDistance;
		}
	}

	public Vision() {
		m_hubEstimator = new MidpointEstimator();
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
		var results = m_camera.getAllUnreadResults();

		for (var result : results) {
			if (result.hasTargets()) {
				List<PhotonTrackedTarget> targets = result.getTargets();

				m_hubEstimator.updateMidpoint(targets);

				SmartDashboard.putNumber("Vision/Angle to Tag", m_hubEstimator.getMidpointAngle());

				SmartDashboard.putNumber(
						"Vision/PoseX", Units.metersToFeet(m_hubEstimator.getMidpointDistance()));
			}

		}
	}

	public double getAngleToHubTag() {
		return m_hubEstimator.getMidpointAngle();
	}

	public double getDistanceToHub() {
		return Units.metersToFeet(m_hubEstimator.getMidpointDistance());
	}
}
