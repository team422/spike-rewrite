package frc.robot.util;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.lib.littletonUtils.AllianceFlipUtil;
import frc.robot.Constants.FieldConstants;

public class ShooterMath {
  // this cannot not get instantiated
  private ShooterMath() {}

  /**
   * Calculates the speed of the top and bottom flywheel based on the robot's position on the field
   *
   * @param pose Current pose of the robot
   * @return The desired velocity of the top and bottom flywheel respectively
   */
  public static Pair<Double, Double> calculateSpeakerFlywheelSpeed(Pose2d pose) {
    return Pair.of(0.0, 0.0);
  }

  public static Rotation2d calculateSpeakerHeading(Pose2d pose) {
    return calculateHeading(
        pose.getTranslation(),
        AllianceFlipUtil.apply(FieldConstants.kCenterSpeakerOpening.toTranslation2d()));
  }

  private static Rotation2d calculateHeading(
      Translation2d robotTranslation, Translation2d targetTranslation) {
    var x = targetTranslation.getMeasureX().minus(robotTranslation.getMeasureX());
    var y = targetTranslation.getMeasureY().minus(robotTranslation.getMeasureY());
    double angle = Math.atan2(y.in(Meters), x.in(Meters));
    return new Rotation2d(Radians.of(angle));
  }
}
