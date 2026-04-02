package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.IntFunction;
import java.util.function.IntToDoubleFunction;
import java.util.function.ToDoubleFunction;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;

public class PoseUtils {
	public record PoseResult(Pose2d pose, double xStdDev, double yStdDev, double thetaStdDev) {
	}

	public record AimResult(double setpoint, double feedforward) {
	}

	private static AprilTagFieldLayout s_layout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

	public static List<Pose2d> extractAllPoses(List<PhotonTrackedTarget> targets, Transform2d transform) {
		int maxResults = targets.size();
		List<Pose2d> poses = new ArrayList<>(maxResults);
		for (PhotonTrackedTarget target : targets) {
			s_layout.getTagPose(target.getFiducialId()).ifPresent(pose -> {
				Pose2d camera = pose.transformBy(target.getBestCameraToTarget().inverse()).toPose2d();
				poses.add(camera.transformBy(transform));
			});
		}
		return poses;
	}

	/**
	 * Estimate the pose of the robot, including the standard deviations for each
	 * component of the pose. This hopefully may be more reliable than other methods
	 * if we want to implement a cutoff which ignores low quality data with high
	 * standard deviations.
	 * 
	 * @param result a {@code PhotonPiplineResult} which has the latest target data
	 * @return a {@code PoseResult} which contains the {@code Pose2d} of the robot
	 *         and the standard deviations
	 */
	public static Optional<PoseResult> estimateCamPoseStdDev(List<Pose2d> poses) {
		if (poses.isEmpty()) {
			return Optional.empty();
		}
		int count = poses.size();
		double[] position = findCenterOfData2d(poses::get, Pose2d::getX, Pose2d::getY, count);
		double[] heading = findCenterOfData2d(
				index -> poses.get(index).getRotation(), Rotation2d::getCos, Rotation2d::getSin, count);
		Pose2d pose = new Pose2d(position[0], position[1], Rotation2d.fromRadians(Math.atan2(heading[1], heading[0])));
		double xStdDev = findStdDevOfData(poses::get, Pose2d::getX, count, position[0]);
		double yStdDev = findStdDevOfData(poses::get, Pose2d::getY, count, position[1]);
		double sinStdDev = findStdDevOfData(
				index -> poses.get(index).getRotation(), Rotation2d::getSin, count, heading[1]);
		double cosStdDev = findStdDevOfData(
				index -> poses.get(index).getRotation(), Rotation2d::getCos, count, heading[0]);
		return Optional.of(new PoseResult(pose, xStdDev, yStdDev, Math.hypot(sinStdDev, cosStdDev)));
	}

	public static <T> double[] findCenterOfData2d(IntFunction<T> itemGetter, ToDoubleFunction<T> xGetter,
			ToDoubleFunction<T> yGetter, int count) {

	}

	public static <T> double[] findCenterOfData2d(IntFunction<T> itemGetter, ToDoubleFunction<T> xGetter,
			ToDoubleFunction<T> yGetter, int count, double meanZone) {

	}

	/**
	 * Finds the standard deviation of a set of data. This function actually
	 * computes the average absolute deviation, but they are essentially the same
	 * for our purposes and standard deviation doesn't work well when there are many
	 * outliers.
	 * 
	 * @param data a way to get the data
	 * @param size how many data points you have
	 * @param center the center of the data
	 * @return the standard deviation of the data
	 */
	public static <T> double findStdDevOfData(IntFunction<T> itemGetter, ToDoubleFunction<T> valueGetter, int size,
			double center) {
		double deviation = 0;
		for (int i = 0; i < size; i++) {
			deviation += Math.abs(data.applyAsDouble(i) - center);
		}
		return deviation / size * Math.sqrt(Math.PI / 2);
	}

	/**
	 * Finds the center of a set of data. Internally this function uses a
	 * bisection algorithm to find the best center.
	 * 
	 * @param data a way to get the data
	 * @param size how many data points you have
	 * @return the best center of the data
	 */
	public static <T> double findCenterOfData(IntFunction<T> itemGetter, ToDoubleFunction<T> valueGetter, int size) {
		return findCenterOfData(data, size, 5);
	}

	/**
	 * Finds the center of a set of data. It has a data-independent tunable
	 * parameter called {@code outlierRejectionAbility} which determines whether the
	 * center is closer to the mean or the median. Internally this function uses a
	 * bisection algorithm to find the best center.
	 * 
	 * @param data a way to get the data
	 * @param size how many data points you have
	 * @param outlierRejectionAbility a parameter to tune what types of centers are
	 *        favored
	 * @return the best center of the data
	 */
	public static <T> double findCenterOfData(IntFunction<T> itemGetter, ToDoubleFunction<T> valueGetter, int size,
			double outlierRejectionAbility) {
		double min = data.applyAsDouble(0), max = min;
		for (int i = 1; i < size; i++) {
			double point = data.applyAsDouble(i);
			max = Math.max(max, point);
			min = Math.min(min, point);
		}
		double k = Math.exp(-outlierRejectionAbility) * (max - min);
		double high = max, low = min, loss;
		do {
			double bisector = (low + high) / 2;
			loss = loss(data, size, bisector, k);
			if (loss < 0) {
				low = bisector;
			} else {
				high = bisector;
			}
		} while (Math.abs(loss) > 0.001);
		return (low + high) / 2;
	}

	/**
	 * Calculates how good of a center the chosen center is for a given set of data.
	 * When k is increased, the function favors the median. When k is decreased, the
	 * function favors the mean.
	 * 
	 * @param data a way to get the data
	 * @param size how many data points you have
	 * @param center a proposed center for the data
	 * @param k a parameter to tune what types of centers are favored
	 * @return how good the center is
	 */
	private static double loss(IntToDoubleFunction data, int size, double center, double k) {
		double total = 0;
		for (int i = 0; i < size; i++) {
			double difference = center - data.applyAsDouble(i);
			total += difference / Math.hypot(difference, k);
		}
		return total;
	}

	private static double[] optimize(IntToDoubleFunction[] data) {
		double[] coordinate = new double[data.length];
	}

	private static double[] slope(double[] coordinate, IntToDoubleFunction[] data) {

	}
}
