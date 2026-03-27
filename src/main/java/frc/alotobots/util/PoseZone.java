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

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.Arrays;
import java.util.List;
import java.util.Objects;
import java.util.Optional;
import java.util.function.Supplier;
import java.util.stream.Collectors;

/**
 * Represents one or more axis-aligned rectangular zones on the field. A PoseZone can be a single
 * box or a composite of multiple boxes. Immutable — all fields are final and no mutation is
 * possible after construction.
 */
public final class PoseZone {

  /**
   * An axis-aligned rectangular box defined by two corners. Corners are automatically normalized on
   * construction so that min contains the smallest x/y and max contains the largest x/y.
   *
   * @param min The normalized minimum corner (smallest x and y).
   * @param max The normalized maximum corner (largest x and y).
   */
  public record Box(Translation2d min, Translation2d max) {
    /**
     * Creates a Box from two corners. Corners are automatically normalized regardless of input
     * order.
     */
    public Box {
      double minX = Math.min(min.getX(), max.getX());
      double minY = Math.min(min.getY(), max.getY());
      double maxX = Math.max(min.getX(), max.getX());
      double maxY = Math.max(min.getY(), max.getY());
      min = new Translation2d(minX, minY);
      max = new Translation2d(maxX, maxY);
    }

    /**
     * @return The center point of the box.
     */
    public Translation2d getCenter() {
      return min.interpolate(max, 0.5);
    }

    /**
     * @return The width (x-axis dimension) of the box.
     */
    public double getWidth() {
      return max.getX() - min.getX();
    }

    /**
     * @return The height (y-axis dimension) of the box.
     */
    public double getHeight() {
      return max.getY() - min.getY();
    }

    /**
     * @return The area of the box (width × height).
     */
    public double getArea() {
      return getWidth() * getHeight();
    }

    /**
     * Checks whether a point is inside this box (inclusive of boundaries).
     *
     * @param point The point to check.
     * @return True if the point is within the box.
     */
    public boolean contains(Translation2d point) {
      return point.getX() >= min.getX()
          && point.getX() <= max.getX()
          && point.getY() >= min.getY()
          && point.getY() <= max.getY();
    }

    /**
     * Returns the closest point inside the box to the given point. If the point is already inside,
     * it is returned as-is.
     *
     * @param point The external point.
     * @return The nearest point within the box.
     */
    public Translation2d closestPoint(Translation2d point) {
      return new Translation2d(
          MathUtil.clamp(point.getX(), min.getX(), max.getX()),
          MathUtil.clamp(point.getY(), min.getY(), max.getY()));
    }

    /**
     * Computes the shortest distance from a point to the box. Returns 0 if the point is inside the
     * box.
     *
     * @param point The point to measure from.
     * @return The shortest distance to the box, or 0 if inside.
     */
    public double distanceTo(Translation2d point) {
      return point.getDistance(closestPoint(point));
    }

    /**
     * Checks whether this box overlaps with another box.
     *
     * @param other The other box.
     * @return True if the boxes share any area.
     */
    public boolean overlaps(Box other) {
      return this.min.getX() <= other.max.getX()
          && this.max.getX() >= other.min.getX()
          && this.min.getY() <= other.max.getY()
          && this.max.getY() >= other.min.getY();
    }

    /**
     * Computes the intersection of this box with another.
     *
     * @param other The other box.
     * @return An Optional containing the overlapping Box, or empty if they don't overlap.
     */
    public Optional<Box> intersection(Box other) {
      if (!overlaps(other)) {
        return Optional.empty();
      }

      Translation2d newMin =
          new Translation2d(
              Math.max(this.min.getX(), other.min.getX()),
              Math.max(this.min.getY(), other.min.getY()));
      Translation2d newMax =
          new Translation2d(
              Math.min(this.max.getX(), other.max.getX()),
              Math.min(this.max.getY(), other.max.getY()));

      return Optional.of(new Box(newMin, newMax));
    }

    /**
     * Flips this box to the other side of the field using PathPlanner's {@link
     * FlippingUtil#flipFieldPosition(Translation2d)}. The resulting box is automatically
     * re-normalized.
     *
     * @return A new Box flipped to the opposite alliance side.
     */
    public Box flip() {
      return new Box(FlippingUtil.flipFieldPosition(min), FlippingUtil.flipFieldPosition(max));
    }
  }

  private final List<Box> boxes;

  private PoseZone(List<Box> boxes) {
    this.boxes = List.copyOf(boxes);
  }

  /**
   * Creates a PoseZone from one or more boxes.
   *
   * @param boxes The boxes that make up this zone.
   * @return A new PoseZone.
   */
  public static PoseZone of(Box... boxes) {
    if (boxes.length == 0) {
      throw new IllegalArgumentException("PoseZone must contain at least one Box.");
    }
    return new PoseZone(Arrays.asList(boxes));
  }

  /**
   * @return An unmodifiable list of boxes that make up this zone.
   */
  public List<Box> getBoxes() {
    return boxes;
  }

  // ---- Containment Checks ----

  /**
   * Checks whether a point is inside any box in this zone.
   *
   * @param point The point to check.
   * @return True if the point is within any box.
   */
  public boolean contains(Translation2d point) {
    return boxes.stream().anyMatch(box -> box.contains(point));
  }

  /**
   * Checks whether a pose's translation is inside any box in this zone. Rotation is ignored.
   *
   * @param pose The pose to check.
   * @return True if the pose's translation is within any box.
   */
  public boolean contains(Pose2d pose) {
    return contains(pose.getTranslation());
  }

  /**
   * Checks whether another PoseZone is fully enclosed within this zone. Every box in the other zone
   * must be fully contained by at least one box in this zone.
   *
   * @param other The other zone to check.
   * @return True if the other zone is entirely within this zone.
   */
  public boolean contains(PoseZone other) {
    return other.boxes.stream()
        .allMatch(
            otherBox ->
                this.boxes.stream()
                    .anyMatch(
                        thisBox ->
                            thisBox.contains(otherBox.min) && thisBox.contains(otherBox.max)));
  }

  // ---- Geometric Utilities ----

  /**
   * @return The total area of all boxes in this zone (overlapping areas are counted multiple
   *     times).
   */
  public double getArea() {
    return boxes.stream().mapToDouble(Box::getArea).sum();
  }

  /**
   * Computes the shortest distance from a point to any box in the zone. Returns 0 if the point is
   * inside any box.
   *
   * @param point The point to measure from.
   * @return The shortest distance to the zone, or 0 if inside.
   */
  public double distanceTo(Translation2d point) {
    return boxes.stream().mapToDouble(box -> box.distanceTo(point)).min().orElse(Double.MAX_VALUE);
  }

  /**
   * Returns the closest point inside the zone to the given point. If the point is already inside
   * any box, it is returned as-is.
   *
   * @param point The external point.
   * @return The nearest point within the zone.
   */
  public Translation2d closestPoint(Translation2d point) {
    return boxes.stream()
        .map(box -> box.closestPoint(point))
        .min((a, b) -> Double.compare(point.getDistance(a), point.getDistance(b)))
        .orElseThrow();
  }

  // ---- Zone Relationships ----

  /**
   * Checks whether this zone overlaps with another zone.
   *
   * @param other The other zone.
   * @return True if any box in this zone overlaps with any box in the other zone.
   */
  public boolean overlaps(PoseZone other) {
    return this.boxes.stream()
        .anyMatch(thisBox -> other.boxes.stream().anyMatch(thisBox::overlaps));
  }

  // ---- Flipping ----

  /**
   * Flips every box in this zone to the other side of the field using PathPlanner's {@link
   * FlippingUtil#flipFieldPosition(Translation2d)}. Respects the currently configured {@link
   * FlippingUtil#symmetryType}.
   *
   * @return A new PoseZone with all boxes flipped to the opposite alliance side.
   */
  public PoseZone flip() {
    List<Box> flippedBoxes = boxes.stream().map(Box::flip).toList();
    return new PoseZone(flippedBoxes);
  }

  // ---- Triggers ----

  /**
   * Creates a {@link Trigger} that is true when the supplied pose is inside this zone.
   *
   * @param poseSupplier A supplier for the robot's current pose (e.g. {@code drivetrain::getPose}).
   * @param loop The event loop to attach the trigger to.
   * @return A Trigger that is true when the pose is inside the zone.
   */
  public Trigger containsTrigger(Supplier<Pose2d> poseSupplier, EventLoop loop) {
    return new Trigger(loop, () -> contains(poseSupplier.get()));
  }

  /**
   * Creates a {@link Trigger} that is true when the supplied pose is inside this zone. Attached to
   * the {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   *
   * @param poseSupplier A supplier for the robot's current pose (e.g. {@code drivetrain::getPose}).
   * @return A Trigger that is true when the pose is inside the zone.
   */
  public Trigger containsTrigger(Supplier<Pose2d> poseSupplier) {
    return containsTrigger(poseSupplier, CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  /**
   * Creates a {@link Trigger} that is true when the supplied pose is within the given distance of
   * this zone.
   *
   * @param poseSupplier A supplier for the robot's current pose.
   * @param distanceMeters The maximum distance in meters for the trigger to be true.
   * @param loop The event loop to attach the trigger to.
   * @return A Trigger that is true when the pose is within the specified distance.
   */
  public Trigger withinDistanceTrigger(
      Supplier<Pose2d> poseSupplier, double distanceMeters, EventLoop loop) {
    return new Trigger(
        loop, () -> distanceTo(poseSupplier.get().getTranslation()) <= distanceMeters);
  }

  /**
   * Creates a {@link Trigger} that is true when the supplied pose is within the given distance of
   * this zone. Attached to the {@link CommandScheduler#getDefaultButtonLoop() default scheduler
   * button loop}.
   *
   * @param poseSupplier A supplier for the robot's current pose.
   * @param distanceMeters The maximum distance in meters for the trigger to be true.
   * @return A Trigger that is true when the pose is within the specified distance.
   */
  public Trigger withinDistanceTrigger(Supplier<Pose2d> poseSupplier, double distanceMeters) {
    return withinDistanceTrigger(
        poseSupplier, distanceMeters, CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  /**
   * Creates a {@link Trigger} that is true when this zone overlaps with another zone. Useful for
   * checking if a dynamically-computed zone intersects a fixed zone.
   *
   * @param otherSupplier A supplier for the other PoseZone to check against.
   * @param loop The event loop to attach the trigger to.
   * @return A Trigger that is true when the zones overlap.
   */
  public Trigger overlapsTrigger(Supplier<PoseZone> otherSupplier, EventLoop loop) {
    return new Trigger(loop, () -> overlaps(otherSupplier.get()));
  }

  /**
   * Creates a {@link Trigger} that is true when this zone overlaps with another zone. Attached to
   * the {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   *
   * @param otherSupplier A supplier for the other PoseZone to check against.
   * @return A Trigger that is true when the zones overlap.
   */
  public Trigger overlapsTrigger(Supplier<PoseZone> otherSupplier) {
    return overlapsTrigger(otherSupplier, CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  // ---- Object Overrides ----

  @Override
  public boolean equals(Object obj) {
    if (this == obj) return true;
    if (!(obj instanceof PoseZone other)) return false;
    return boxes.equals(other.boxes);
  }

  @Override
  public int hashCode() {
    return Objects.hash(boxes);
  }

  @Override
  public String toString() {
    String boxStr =
        boxes.stream()
            .map(box -> String.format("Box(min=%s, max=%s)", box.min(), box.max()))
            .collect(Collectors.joining(", "));
    return String.format("PoseZone[%s]", boxStr);
  }
}
