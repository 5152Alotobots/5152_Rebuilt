/*
* ALOTOBOTS - FRC Team 5152
  https://github.com/5152Alotobots
* Copyright (C) 2026 ALOTOBOTS
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Source code must be publicly available on GitHub or an alternative web accessible site
*/
package frc.alotobots.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import java.util.Objects;

import static edu.wpi.first.units.Units.Radians;

/**
 * A {@link Rotation2d} subclass that constrains its angle to an arbitrary [lowerBound, upperBound)
 * range instead of the standard (-π, π].
 *
 * <p>Behavior adapts based on the span of the range:
 *
 * <ul>
 *   <li><b>Span = 360°:</b> Pure wrapping via {@link MathUtil#inputModulus}. Equivalent to shifting
 *       where the discontinuity lives.
 *   <li><b>Span &lt; 360° (gap):</b> Wraps to the nearest full rotation, then <em>clamps</em> to
 *       whichever bound is closer if the angle lands in the dead zone. Ideal for mechanisms with
 *       physical hard stops.
 *   <li><b>Span &gt; 360° (overlap):</b> Stores the raw value without wrapping as long as it stays
 *       within bounds. Only applies {@code inputModulus} when the value actually exceeds the range.
 *       Useful for multi-rotation tracking with soft limits.
 * </ul>
 *
 * <h2>Example</h2>
 *
 * <pre>{@code
 * // Standard shifted wrap: [-125°, 235°)
 * var a = BoundedRotation2d.fromDegrees(-125, 235, 270);
 * a.getDegrees(); // -90.0
 *
 * // Hard-stop turret: [-165°, 165°) — 330° span, 30° dead zone in the back
 * var b = BoundedRotation2d.fromDegrees(-165, 165, 180);
 * b.getDegrees(); // 165.0 (clamped to nearest bound)
 *
 * // Multi-rotation: [-180°, 540°) — 720° span
 * var c = BoundedRotation2d.fromDegrees(-180, 540, 400);
 * c.getDegrees(); // 400.0 (raw, still in bounds)
 * }</pre>
 */
public class BoundedRotation2d extends Rotation2d {

  private static final double TWO_PI = 2.0 * Math.PI;
  private static final double EPSILON = 1.0e-9;

  private final double m_lowerBoundRad;
  private final double m_upperBoundRad;
  private final double m_spanRad;

  // ———————————————————————————————————————————
  //  Core constraining logic
  // ———————————————————————————————————————————

  /** Constrains a radian value into [lower, upper) using the strategy appropriate for the span. */
  private static double constrain(double radians, double lowerRad, double upperRad) {
    double span = upperRad - lowerRad;

    if (span > TWO_PI + EPSILON) {
      // Overlap (>360°): keep raw value if already in bounds
      if (radians >= lowerRad && radians < upperRad) {
        return radians;
      }
      return MathUtil.inputModulus(radians, lowerRad, upperRad);
    }

    if (span < TWO_PI - EPSILON) {
      // Gap (<360°): wrap to full rotation centered on range, then clamp
      double center = (lowerRad + upperRad) / 2.0;
      double wrapped = MathUtil.inputModulus(radians, center - Math.PI, center + Math.PI);
      return MathUtil.clamp(wrapped, lowerRad, upperRad);
    }

    // Exact 360°: pure modular wrap
    double wrapped = MathUtil.inputModulus(radians, lowerRad, upperRad);
    // inputModulus can return upperRad on exact boundaries.
    // Snap to lowerRad to enforce [lower, upper) semantics.
    if (Math.abs(wrapped - upperRad) < EPSILON) {
      wrapped = lowerRad;
    }
    return wrapped;
  }

  // ———————————————————————————————————————————
  //  Constructors
  // ———————————————————————————————————————————

  /**
   * Constructs a BoundedRotation2d from radian bounds and a radian value.
   *
   * @param lowerBoundRad Lower bound of the range (radians, inclusive).
   * @param upperBoundRad Upper bound of the range (radians, exclusive).
   * @param radians The angle in radians (will be constrained into range).
   */
  public BoundedRotation2d(double lowerBoundRad, double upperBoundRad, double radians) {
    super(constrain(radians, lowerBoundRad, upperBoundRad));
    validateBounds(lowerBoundRad, upperBoundRad);
    this.m_lowerBoundRad = lowerBoundRad;
    this.m_upperBoundRad = upperBoundRad;
    this.m_spanRad = upperBoundRad - lowerBoundRad;
  }

  /**
   * Constructs a BoundedRotation2d from {@link Angle} bounds and value.
   *
   * @param lowerBound Lower bound (inclusive).
   * @param upperBound Upper bound (exclusive).
   * @param angle The angle (will be constrained into range).
   */
  public BoundedRotation2d(Angle lowerBound, Angle upperBound, Angle angle) {
    this(lowerBound.in(Radians), upperBound.in(Radians), angle.in(Radians));
  }

  /**
   * Constructs a BoundedRotation2d at zero (or its constrained equivalent).
   *
   * @param lowerBoundRad Lower bound (radians, inclusive).
   * @param upperBoundRad Upper bound (radians, exclusive).
   */
  public BoundedRotation2d(double lowerBoundRad, double upperBoundRad) {
    this(lowerBoundRad, upperBoundRad, 0.0);
  }

  /**
   * Constructs a BoundedRotation2d from an x, y direction vector.
   *
   * @param lowerBoundRad Lower bound (radians, inclusive).
   * @param upperBoundRad Upper bound (radians, exclusive).
   * @param x The x component of the direction.
   * @param y The y component of the direction.
   */
  public BoundedRotation2d(double lowerBoundRad, double upperBoundRad, double x, double y) {
    this(lowerBoundRad, upperBoundRad, Math.atan2(y, x));
  }

  // ———————————————————————————————————————————
  //  Static Factories
  // ———————————————————————————————————————————

  public static BoundedRotation2d fromDegrees(
      double lowerBoundDeg, double upperBoundDeg, double degrees) {
    return new BoundedRotation2d(
        Math.toRadians(lowerBoundDeg), Math.toRadians(upperBoundDeg), Math.toRadians(degrees));
  }

  public static BoundedRotation2d fromRadians(
      double lowerBoundRad, double upperBoundRad, double radians) {
    return new BoundedRotation2d(lowerBoundRad, upperBoundRad, radians);
  }

  public static BoundedRotation2d fromRotations(
      double lowerBoundRot, double upperBoundRot, double rotations) {
    return new BoundedRotation2d(
        edu.wpi.first.math.util.Units.rotationsToRadians(lowerBoundRot),
        edu.wpi.first.math.util.Units.rotationsToRadians(upperBoundRot),
        edu.wpi.first.math.util.Units.rotationsToRadians(rotations));
  }

  public static BoundedRotation2d fromMeasure(Angle lowerBound, Angle upperBound, Angle angle) {
    return new BoundedRotation2d(lowerBound, upperBound, angle);
  }

  public static BoundedRotation2d fromRotation2d(
      double lowerBoundRad, double upperBoundRad, Rotation2d rotation) {
    return new BoundedRotation2d(lowerBoundRad, upperBoundRad, rotation.getRadians());
  }

    public static BoundedRotation2d fromRotation2d(
            Angle lowerBound, Angle upperBound, Rotation2d rotation) {
        return new BoundedRotation2d(lowerBound.in(Radians), upperBound.in(Radians), rotation.getRadians());
    }

  // ———————————————————————————————————————————
  //  Internal helper
  // ———————————————————————————————————————————

  private BoundedRotation2d rewrap(double radians) {
    return new BoundedRotation2d(m_lowerBoundRad, m_upperBoundRad, radians);
  }

  // ———————————————————————————————————————————
  //  Arithmetic — covariant returns
  // ———————————————————————————————————————————

  @Override
  public BoundedRotation2d plus(Rotation2d other) {
    return rewrap(getRadians() + other.getRadians());
  }

  @Override
  public BoundedRotation2d minus(Rotation2d other) {
    return rewrap(getRadians() - other.getRadians());
  }

  @Override
  public BoundedRotation2d unaryMinus() {
    return rewrap(-getRadians());
  }

  @Override
  public BoundedRotation2d times(double scalar) {
    return rewrap(getRadians() * scalar);
  }

  @Override
  public BoundedRotation2d div(double scalar) {
    return times(1.0 / scalar);
  }

  @Override
  public BoundedRotation2d rotateBy(Rotation2d other) {
    return rewrap(super.rotateBy(other).getRadians());
  }

  @Override
  public BoundedRotation2d relativeTo(Rotation2d other) {
    return rewrap(super.relativeTo(other).getRadians());
  }

  // ———————————————————————————————————————————
  //  Interpolation (covariant override)
  // ———————————————————————————————————————————

  /**
   * Interpolates between this rotation and an end value.
   *
   * <p>For spans ≤ 360°, uses shortest-path through the valid range. For spans &gt; 360°, linearly
   * interpolates the raw values.
   *
   * @param endValue The target rotation (will be constrained into this range).
   * @param t Interpolation parameter, clamped to [0, 1].
   * @return The interpolated BoundedRotation2d.
   */
  @Override
  public BoundedRotation2d interpolate(Rotation2d endValue, double t) {
    double clamped = MathUtil.clamp(t, 0.0, 1.0);
    double endConstrained = constrain(endValue.getRadians(), m_lowerBoundRad, m_upperBoundRad);

    if (m_spanRad > TWO_PI) {
      // Overlap: straight lerp in the raw value space — no angular wrap tricks
      return rewrap(this.getRadians() + (endConstrained - this.getRadians()) * clamped);
    }

    // For ≤360° spans, find shortest path within the effective angular range.
    // The effective wrap width is min(span, 2π).
    double effectiveSpan = Math.min(m_spanRad, TWO_PI);
    double halfSpan = effectiveSpan / 2.0;
    double delta = MathUtil.inputModulus(endConstrained - this.getRadians(), -halfSpan, halfSpan);
    return rewrap(this.getRadians() + delta * clamped);
  }

  // ———————————————————————————————————————————
  //  Query helpers
  // ———————————————————————————————————————————

  /**
   * Returns whether the constrained range has a gap (span &lt; 360°).
   *
   * @return true if angles exist that are outside the valid range.
   */
  public boolean hasGap() {
    return m_spanRad < TWO_PI - EPSILON;
  }

  /**
   * Returns whether the constrained range has overlap (span &gt; 360°).
   *
   * @return true if some physical angles have multiple valid representations.
   */
  public boolean hasOverlap() {
    return m_spanRad > TWO_PI + EPSILON;
  }

  /**
   * Returns whether this rotation is currently sitting at one of the clamped bounds (only
   * meaningful when {@link #hasGap()} is true).
   *
   * @return true if the value equals the lower or upper bound.
   */
  public boolean isAtLimit() {
    return Math.abs(getRadians() - m_lowerBoundRad) < EPSILON
        || Math.abs(getRadians() - m_upperBoundRad) < EPSILON;
  }

  // ———————————————————————————————————————————
  //  Bound accessors
  // ———————————————————————————————————————————

  public double getLowerBoundRadians() {
    return m_lowerBoundRad;
  }

  public double getUpperBoundRadians() {
    return m_upperBoundRad;
  }

  public double getLowerBoundDegrees() {
    return Math.toDegrees(m_lowerBoundRad);
  }

  public double getUpperBoundDegrees() {
    return Math.toDegrees(m_upperBoundRad);
  }

  public Angle getLowerBoundMeasure() {
    return Radians.of(m_lowerBoundRad);
  }

  public Angle getUpperBoundMeasure() {
    return Radians.of(m_upperBoundRad);
  }

  public double getSpanRadians() {
    return m_spanRad;
  }

  public double getSpanDegrees() {
    return Math.toDegrees(m_spanRad);
  }

  // ———————————————————————————————————————————
  //  Conversion
  // ———————————————————————————————————————————

  /** Returns a plain {@link Rotation2d} (discards bounds). */
  public Rotation2d toRotation2d() {
    return new Rotation2d(getRadians());
  }

  // ———————————————————————————————————————————
  //  Object overrides
  // ———————————————————————————————————————————

  @Override
  public String toString() {
    return String.format(
        "BoundedRotation2d(Rads: %.2f, Deg: %.2f, Range: [%.2f°, %.2f°), Span: %.2f°)",
        getRadians(),
        getDegrees(),
        getLowerBoundDegrees(),
        getUpperBoundDegrees(),
        getSpanDegrees());
  }

  @Override
  public boolean equals(Object obj) {
    return super.equals(obj);
  }

  @Override
  public int hashCode() {
    return Objects.hash(getRadians());
  }

  // ———————————————————————————————————————————
  //  Validation
  // ———————————————————————————————————————————

  private static void validateBounds(double lowerRad, double upperRad) {
    if (upperRad <= lowerRad) {
      throw new IllegalArgumentException(
          String.format(
              "Upper bound (%.4f rad / %.2f°) must be greater than lower bound (%.4f rad / %.2f°)",
              upperRad, Math.toDegrees(upperRad), lowerRad, Math.toDegrees(lowerRad)));
    }
  }
}
