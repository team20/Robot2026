package frc.robot;

import java.util.List;
import java.util.stream.Stream;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Filter {
	public static class DoubleExponential {
		private enum State {
			EMPTY,
			UNINITIALIZED,
			INITIALIZED;
		}

		private final double m_alpha;
		private final double m_beta;
		private double m_value;
		private double m_trend;
		private State m_state = State.EMPTY;

		public DoubleExponential(double alpha, double beta) {
			m_alpha = alpha;
			m_beta = beta;
		}

		public double calculate(double input) {
			switch (m_state) {
				case EMPTY -> {
					m_value = input;
					m_state = State.UNINITIALIZED;
				}
				case INITIALIZED -> {
					double last = m_value;
					m_value = m_alpha * input + (1 - m_alpha) * (m_value + m_trend);
					m_trend = m_beta * (m_value - last) + (1 - m_beta) * m_trend;
				}
				case UNINITIALIZED -> {
					m_trend = input - m_value;
					m_value = input;
					m_state = State.INITIALIZED;
				}
			}
			return m_value;
		}

		public double predict() {
			return m_value + m_trend;
		}
	}

	public static class RejectionFilter {
		private final DoubleExponential m_valueFilter;
		private final DoubleExponential m_deviationFilter;

		public RejectionFilter(double alpha, double beta) {
			m_valueFilter = new DoubleExponential(alpha, beta);
			m_deviationFilter = new DoubleExponential(alpha, beta);
		}

		public double calculate(double input) {
			double predictedValue = m_valueFilter.predict();
			double predictedDeviation = m_deviationFilter.predict();
			double calculatedValue = m_valueFilter.calculate(input);
			m_deviationFilter.calculate(Math.abs(calculatedValue - input));
			if (Math.abs(predictedValue - input) > predictedDeviation) {
				return predictedValue;
			} else {
				return calculatedValue;
			}
		}
	}

	public static Command testCommand() {
		String dataset = """
				1167.60,-0.50
				1167.75,-0.50
				1167.90,-0.52
				1168.05,-0.50
				1168.10,-0.10
				1168.12,-0.62
				1168.25,-0.48
				1168.35,-0.52
				1168.45,-0.20
				1168.47,-0.78
				1168.60,-0.50
				1168.75,-0.52
				1168.90,-0.15
				1168.92,-0.60
				1169.05,-0.50
				1169.20,-0.52
				1169.30,-0.18
				1169.32,-0.70
				1169.50,-0.50
				1169.65,-0.55
				1169.75,-0.22
				1169.77,-0.65
				1169.90,-0.50
				1170.05,-0.52
				1170.15,-0.30
				1170.17,-0.72
				1170.30,-0.50
				1170.45,-0.52
				1170.55,-0.28
				1170.57,-0.68
				1170.70,-0.50
				1170.85,-0.55
				1170.95,-0.25
				1170.97,-0.75
				1171.10,-0.50
				1171.25,-0.52
				1171.35,-0.18
				1171.37,-0.70
				1171.50,-0.50
				1171.65,-0.53
				1171.78,-0.30
				1171.80,-0.78
				1171.95,-0.50
				1172.10,-0.52
				1172.25,-0.28
				1172.27,-0.60
				1172.40,-0.50
				1172.55,-0.48
				1172.70,-0.20
				1172.72,-0.58
				1172.85,-0.10
				1173.00,-0.05""";
		return Commands.runOnce(() -> {
			LinearFilter movingAverage = LinearFilter.movingAverage(6);
			MedianFilter median = new MedianFilter(6);
			DoubleExponential doubleExponential = new DoubleExponential(0.05, 0.05);
			RejectionFilter rejection = new RejectionFilter(0.05, 0.05);
			for (String datapoint : dataset.split("\n")) {
				List<Double> values = Stream.of(datapoint.split(",")).map(Double::parseDouble).toList();
				double movingAverageValue = movingAverage.calculate(values.get(1));
				double medianValue = median.calculate(values.get(1));
				double doubleExponentialValue = doubleExponential.calculate(values.get(1));
				double rejectionValue = rejection.calculate(values.get(1));
				System.out.printf(
						"%f,%f,%f,%f,%f,%f%n", values.get(0), values.get(1), movingAverageValue, medianValue,
						doubleExponentialValue, rejectionValue);
			}
		});
	}
}
