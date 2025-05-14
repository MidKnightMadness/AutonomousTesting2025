package org.firstinspires.ftc.teamcode.Components;

public class InversePolynomialApproximator {
    private final PolynomialApproximator approximator;
    private final double tolerance;
    private final int maxIterations;

    public InversePolynomialApproximator(PolynomialApproximator approximator, double tolerance, int maxIterations) {
        this.approximator = approximator;
        this.tolerance = tolerance;
        this.maxIterations = maxIterations;
    }

    public double inverseEvaluate(double y) {
        double a = approximator.getDomainMin();
        double b = approximator.getDomainMax();
        double fa = approximator.evaluate(a) - y;
        double fb = approximator.evaluate(b) - y;

        if (fa * fb > 0) {
            throw new IllegalArgumentException("Function does not have opposite signs at the interval endpoints.");
        }

        double c = a;
        for (int i = 0; i < maxIterations; i++) {
            c = (a + b) / 2;
            double fc = approximator.evaluate(c) - y;

            if (Math.abs(fc) < tolerance) {
                return c;
            }

            if (fa * fc < 0) {
                b = c;
                fb = fc;
            } else {
                a = c;
                fa = fc;
            }
        }

        return c;
    }

    @Override
    public String toString() {
        return "InversePolynomialApproximator for: " + approximator.toString();
    }
}

