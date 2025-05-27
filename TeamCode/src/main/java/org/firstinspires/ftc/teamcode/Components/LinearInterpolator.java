package org.firstinspires.ftc.teamcode.Components;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

/**
 * A simple linear interpolator for a set of (x, y) points.
 * Points are sorted by x-coordinate.  Evaluation is done by finding the interval
 * that contains the query x and linearly interpolating between the two bounding points.
 */
public class LinearInterpolator {
    /** Represents a 2D point with x and y coordinates. */
    public static class Point {
        public final double x;
        public final double y;

        public Point(double x, double y) {
            this.x = x;
            this.y = y;
        }
    }

    private final List<Point> points;

    /**
     * Constructs a LinearInterpolator with the given list of points.
     * At least two points are required.
     * Points will be sorted by increasing x internally.
     *
     * @param inputPoints List of Point objects
     * @throws IllegalArgumentException if fewer than two points are provided
     */
    public LinearInterpolator(List<Point> inputPoints) {
        if (inputPoints == null || inputPoints.size() < 2) {
            throw new IllegalArgumentException("At least two points are required for interpolation.");
        }
        // Copy and sort the points by x-coordinate
        points = new ArrayList<>(inputPoints);
        Collections.sort(points, Comparator.comparingDouble(p -> p.x));
    }

    /**
     * Evaluates the interpolated (or extrapolated) y value at the given x coordinate.
     * If x is outside the range of input points, linear extrapolation is performed
     * using the first or last segment.
     *
     * @param x the x-coordinate to evaluate
     * @return interpolated (or extrapolated) y value
     */
    public double evaluate(double x) {
        // If x is before the first point, extrapolate using first segment
        if (x <= points.get(0).x) {
            return interpolate(points.get(0), points.get(1), x);
        }
        // If x is after the last point, extrapolate using last segment
        int lastIndex = points.size() - 1;
        if (x >= points.get(lastIndex).x) {
            return interpolate(points.get(lastIndex - 1), points.get(lastIndex), x);
        }
        // Otherwise, find the interval [x_i, x_{i+1}] that contains x
        for (int i = 0; i < lastIndex; i++) {
            Point p0 = points.get(i);
            Point p1 = points.get(i + 1);
            if (x >= p0.x && x <= p1.x) {
                return interpolate(p0, p1, x);
            }
        }
        // This should never happen if the above logic is correct
        throw new IllegalStateException("Failed to interpolate for x = " + x);
    }

    /**
     * Helper method to perform linear interpolation/extrapolation between two points.
     *
     * @param p0 the first point
     * @param p1 the second point
     * @param x the x-coordinate to interpolate at
     * @return the interpolated y value
     */
    private double interpolate(Point p0, Point p1, double x) {
        double t = (x - p0.x) / (p1.x - p0.x);
        return p0.y + t * (p1.y - p0.y);
    }

    public static LinearInterpolator rightAxonVoltageInterpolator() {

        List<Point> rightPoints = List.of(
                new Point(0.10, 0.0349),
                new Point(0.20, 0.1771),
                new Point(0.30, 0.3840),
                new Point(0.40, 0.2251),
                new Point(0.50, 0.1835),
                new Point(0.60, 0.2116),
                new Point(0.70, 0.2176),
                new Point(0.80, 0.2330),
                new Point(0.90, 0.2087),
                new Point(1.00, 0.2502),
                new Point(0.25, 0.2242),
                new Point(0.35, 0.4864),
                new Point(0.375, 0.5353),
                new Point(0.38, 0.5410),
                new Point(0.39, 0.5250),
                new Point(0.395, 0.3304),
                new Point(0.45, 0.2395)
        );

        return new LinearInterpolator(rightPoints);

    }

    public static LinearInterpolator leftAxonVoltageInterpolator() {
        List<Point> leftPoints = new ArrayList<>();
        leftPoints.add(new Point(0.10, 0.0682));
        leftPoints.add(new Point(0.20, 0.2605));
        leftPoints.add(new Point(0.25, 0.3110));
        leftPoints.add(new Point(0.30, 0.5230));
        leftPoints.add(new Point(0.35, 0.5962));
        leftPoints.add(new Point(0.375, 0.5796));
        leftPoints.add(new Point(0.38, 0.5747));
        leftPoints.add(new Point(0.39, 0.5590));
        leftPoints.add(new Point(0.395, 0.4089));
        leftPoints.add(new Point(0.40, 0.3170));
        leftPoints.add(new Point(0.45, 0.3361));
        leftPoints.add(new Point(0.50, 0.3035));
        leftPoints.add(new Point(0.60, 0.3225));
        leftPoints.add(new Point(0.70, 0.3260));
        leftPoints.add(new Point(0.80, 0.3404));
        leftPoints.add(new Point(0.90, 0.3215));
        leftPoints.add(new Point(1.00, 0.3530));

        return new LinearInterpolator(leftPoints);
    }
}

