package frc.robot;

import java.util.function.DoubleUnaryOperator;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.util.DoubleCircularBuffer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class SensorFusion {
	private final DoubleCircularBuffer m_accurateSignal;
	private final DoubleCircularBuffer m_consistentSignal;
	private final int m_size;
	private final double m_dt;
	private double m_derivative;
	private double m_value;

	public SensorFusion(int windowSize, double dt) {
		m_dt = dt;
		m_size = windowSize;
		m_accurateSignal = new DoubleCircularBuffer(m_size);
		m_consistentSignal = new DoubleCircularBuffer(m_size);
	}

	public void update(double accurateValue, double consistentValue) {
		m_accurateSignal.addFirst(accurateValue);
		m_consistentSignal.addFirst(consistentValue);
		SimpleMatrix systemCoefficients = new SimpleMatrix(m_size, 3);
		SimpleMatrix consistent = new SimpleMatrix(m_size, 1);
		for (int i = 0; i < m_size; i++) {
			systemCoefficients.setRow(i, 0, i * i, i, 1);
			consistent.set(i, 0, m_consistentSignal.get(i));
		}
		SimpleMatrix transpose = systemCoefficients.transpose();
		SimpleMatrix quadraticCoefficients = transpose.mult(systemCoefficients).invert().mult(transpose)
				.mult(consistent);
		SimpleMatrix consistentFiltered = systemCoefficients.mult(quadraticCoefficients);
		systemCoefficients = new SimpleMatrix(m_size, 2);
		SimpleMatrix accurate = new SimpleMatrix(m_size, 1);
		for (int i = 0; i < m_size; i++) {
			systemCoefficients.setRow(i, 0, consistentFiltered.get(i, 0), 1);
			accurate.set(i, 0, m_accurateSignal.get(i));
		}
		transpose = systemCoefficients.transpose();
		SimpleMatrix linearCoefficients = transpose.mult(systemCoefficients).invert().mult(transpose).mult(accurate);
		m_value = linearCoefficients.get(0, 0) * quadraticCoefficients.get(2, 0) + linearCoefficients.get(1, 0);
		m_derivative = linearCoefficients.get(0, 0) * quadraticCoefficients.get(1, 0) / m_dt;
	}

	public double getDerivative() {
		return m_derivative;
	}

	public double getValue() {
		return m_value;
	}

	public static Command testCommand() {
		return Commands.runOnce(() -> {
			double dt = 0.01;
			SensorFusion fusion = new SensorFusion(30, dt);
			DoubleUnaryOperator f = x -> x - Math.floor(x);
			DoubleUnaryOperator r = f.compose(x -> (1000 * Math.sin(1000 * x)));
			DoubleUnaryOperator y1 = x -> Math.sin(x) + 0.05 * r.applyAsDouble(x);
			DoubleUnaryOperator y2 = x -> 1.3 * y1.applyAsDouble(x) - 0.2 + 0.3 * r.applyAsDouble(x);
			for (double t = 0; t <= 10; t += dt) {
				double consistent = y1.applyAsDouble(t);
				double accurate = y2.applyAsDouble(t);
				fusion.update(accurate, consistent);
				System.out.printf("%f,%f,%f\n", t, fusion.getValue(), fusion.getDerivative());
			}
		});
	}
}
