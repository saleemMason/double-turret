package frc.robot;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;

public class FieldLocations {
  /** Field dimension along the X axis */
  public static final Distance fieldLength = Inches.of(651.22);

  /** Field dimension along the Y axis */
  public static final Distance fieldWidth = Inches.of(317.69);

  public static final Distance fieldCenterX = fieldLength.div(2);
  public static final Distance fieldCenterY = fieldWidth.div(2);

  public static final Distance blueHubCenterX = Inches.of(182.11);
  public static final Translation2d blueHubCenter = new Translation2d(blueHubCenterX, fieldCenterY);

  public static final Distance redHubCenterX = fieldLength.minus(blueHubCenterX);
  public static final Translation2d redHubCenter = new Translation2d(redHubCenterX, fieldCenterY);

  /** Field dimension along the X axis */
  public static final Distance bumpLength = Inches.of(47);

  /** Field dimension along the Y axis */
  public static final Distance bumpWidth = Inches.of(50.34);
}
