package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Subsystems.VisionConstants;

public class Vision extends SubsystemBase {
	private static Vision s_vision;
	private final PhotonCamera m_camera;
	private double m_distanceToHub, m_angleToHubTag;

	public Vision() {
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
				double distanceSum = 0;
				double angleSum = 0;
				double totalWeight = 0;
				double numTags = 0;
				for (PhotonTrackedTarget target : targets) {
					if (!VisionConstants.kTrackableTags.contains(target.fiducialId))
						continue;

					numTags++;
					double weight = Math.pow(target.getArea(), 2) * (1 -
							target.getPoseAmbiguity());
					totalWeight += weight;

					Transform3d botPose = target.getBestCameraToTarget();

					angleSum += target.yaw; // * weight;

					distanceSum += (botPose.getX() +
							Units.inchesToMeters(23.5));// * Math.sqrt(4 / Math.PI))) * weight;
				}

				m_distanceToHub = distanceSum / numTags;// totalWeight
				m_angleToHubTag = angleSum / numTags;// totalWeight

				SmartDashboard.putNumber("Vision/Angle to Tag", m_angleToHubTag);

				SmartDashboard.putNumber(
						"Vision/PoseX", Units.metersToFeet(m_distanceToHub));
			}

		}
	}

	public double getAngleToHubTag() {
		return m_angleToHubTag;
	}

	public double getDistanceToHub() {
		return Units.metersToFeet(m_distanceToHub);
	}
}
