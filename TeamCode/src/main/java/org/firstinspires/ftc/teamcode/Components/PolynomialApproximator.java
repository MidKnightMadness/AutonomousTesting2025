package org.firstinspires.ftc.teamcode.Components;

import org.firstinspires.ftc.teamcode.RunOptions;

public class PolynomialApproximator {
    private final double[] coefficients;
    private final double domainMin;
    private final double domainMax;

    public static boolean isDevelopment = RunOptions.isDevelopment;

    public PolynomialApproximator(double[] coefficients, double domainMin, double domainMax) {
        if (coefficients == null || coefficients.length == 0) {
            throw new IllegalArgumentException("Coefficient array must not be null or empty.");
        }
        if (domainMin > domainMax) {
            throw new IllegalArgumentException("domainMin must be less than or equal to domainMax.");
        }
        this.coefficients = coefficients.clone();
        this.domainMin = domainMin;
        this.domainMax = domainMax;
    }

    public double evaluate(double x) {
        double clampedX = x;

        if (x < domainMin || x > domainMax) {
            if (isDevelopment) {
                throw new IllegalArgumentException("Input x = " + x + " is outside the domain [" + domainMin + ", " + domainMax + "].");
            }
            else {
                clampedX = Math.max(domainMin, Math.min(x, domainMax));
            }
        }

        double result = 0.0;

        // Horner's method
        for (int i = coefficients.length - 1; i >= 0; i--) {
            result = result * clampedX + coefficients[i];
        }
        return result;
    }

    public double[] getCoefficients() {
        return coefficients.clone();
    }

    public double getDomainMin() {
        return domainMin;
    }

    public double getDomainMax() {
        return domainMax;
    }

    public int degree() {
        return coefficients.length - 1;
    }

    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder();
        sb.append("PolynomialApproximator: ");
        for (int i = coefficients.length - 1; i >= 0; i--) {
            double coef = coefficients[i];
            if (coef == 0) continue;

            // Determine the sign
            if (sb.length() > 24) {
                sb.append(coef > 0 ? " + " : " - ");
            } else if (coef < 0) {
                sb.append("-");
            }

            // Format the coefficient
            double absCoef = Math.abs(coef);
            String formattedCoef;
            if (absCoef >= 1e-4) {
                formattedCoef = String.format("%.4f", absCoef);
            } else {
                formattedCoef = String.format("%.4e", absCoef);
            }

            sb.append(formattedCoef);

            // Append variable part
            if (i > 0) sb.append("x");
            if (i > 1) sb.append("^").append(i);
        }
        sb.append(" on domain [").append(domainMin).append(", ").append(domainMax).append("]");
        return sb.toString();
    }

}

