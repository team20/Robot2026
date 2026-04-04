package frc.robot;

import java.util.Arrays;
import java.util.function.DoubleUnaryOperator;
import java.util.function.IntFunction;
import java.util.function.IntToDoubleFunction;
import java.util.function.ToDoubleFunction;

public class DataUtils {
	public static double minAngularDifference(double a, double b) {
		double difference = Math.abs(a - b);
		return Math.min(difference, 360 - difference);
	}

	public static double linearInterpolation(double a, double b, double t) {
		return (1 - t) * a + t * b;
	}

	public static DoubleUnaryOperator standardWFunction(double meanRadius) {
		return radius -> 2 * meanRadius / radius * (1 - Math.exp(-radius / meanRadius));
	}

	private static final int ITERAITONS = 10;

	@SuppressWarnings("unchecked")
	public static <T> double[] findMultidimensionalCenter(int nPoints, DoubleUnaryOperator wFunction,
			IntFunction<T> item, ToDoubleFunction<T>... data) {
		IntToDoubleFunction[] combinedData = new IntToDoubleFunction[data.length];
		for (int i = 0; i < data.length; i++) {
			int j = i;
			combinedData[i] = index -> data[j].applyAsDouble(item.apply(index));
		}
		return findMultidimensionalCenter(nPoints, wFunction, combinedData);
	}

	public static double[] findMultidimensionalCenter(int nPoints, DoubleUnaryOperator wFunction,
			IntToDoubleFunction... data) {
		double[] center = new double[data.length];
		for (int i = 0; i < data.length; i++) {
			for (int j = 0; j < nPoints; j++) {
				center[i] += data[i].applyAsDouble(j);
			}
			center[i] /= nPoints;
		}
		if (nPoints == 1) {
			return center;
		}
		double[] numerator = new double[data.length];
		double denominator;
		for (int i = 0; i < ITERAITONS; i++) {
			Arrays.fill(numerator, 0);
			denominator = 0;
			for (int j = 0; j < nPoints; j++) {
				double squaredError = 0;
				for (int k = 0; k < data.length; k++) {
					squaredError += Math.pow(center[k] - data[k].applyAsDouble(j), 2);
				}
				double w = wFunction.applyAsDouble(Math.sqrt(squaredError));
				for (int k = 0; k < data.length; k++) {
					numerator[k] += w * data[k].applyAsDouble(j);
				}
				denominator += w;
			}
			for (int j = 0; j < data.length; j++) {
				center[j] = numerator[j] / denominator;
			}
		}
		return center;
	}

	/**
	 * Finds the average absolute deviation of a set of data. This function is not
	 * quite similar to computing the standard deviation, but they are essentially
	 * the same for our purposes and standard deviation doesn't work well when there
	 * are many outliers.
	 * 
	 * @param data a way to get the data
	 * @param size how many data points you have
	 * @param center the center of the data
	 * @return the standard deviation of the data
	 */
	public static <T> double[] findAverageAbsoluteDeviation(int nPoints, double[] center, IntToDoubleFunction... data) {
		for (int i = 0; i < data.length; i++) {
			double deviation = 0;
			for (int k = 0; k < nPoints; k++) {
				deviation += Math.abs(data[i].applyAsDouble(k) - center[i]);
			}
			center[i] = deviation / nPoints * Math.sqrt(Math.PI / 2);
		}
		return center;
	}

	/**
	 * Finds the average absolute deviation of a set of data. This function is not
	 * quite similar to computing the standard deviation, but they are essentially
	 * the same for our purposes and standard deviation doesn't work well when there
	 * are many outliers.
	 * 
	 * @param data a way to get the data
	 * @param size how many data points you have
	 * @param center the center of the data
	 * @return the standard deviation of the data
	 */
	@SuppressWarnings("unchecked")
	public static <T> double[] findAverageAbsoluteDeviation(int nPoints, double[] center,
			IntFunction<T> item, ToDoubleFunction<T>... data) {
		IntToDoubleFunction[] combinedData = new IntToDoubleFunction[data.length];
		for (int i = 0; i < data.length; i++) {
			int j = i;
			combinedData[i] = index -> data[j].applyAsDouble(item.apply(index));
		}
		return findAverageAbsoluteDeviation(nPoints, center, combinedData);
	}
}
