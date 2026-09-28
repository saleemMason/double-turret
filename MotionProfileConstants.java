// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionConstants {
  public static record CameraConstants(
      /** Camera name, must match name configured on coprocessor */
      String name,
      /** Robot to camera transforms */
      Transform3d robotToCamera,
      /**
       * Standard deviation multipliers for each camera (Adjust to trust some cameras more than
       * others)
       */
      double stdDevFactor,
      /** List of valid apriltag ids. All other apriltags will not be used for pose estimation. */
      long[] fidicualFilter,
      /**
       * Number of frames to skip between each processed frame while robot is disabled Reduces
       * temperature rise on limelight cameras while disabled
       */
      double disabledThrottleFrames) {
    public CameraConstants(
        String name, Transform3d robotToCamera, double stdDevFactor, long[] fidicualFilter) {
      this(name, robotToCamera, stdDevFactor, fidicualFilter, 100);
    }

    public CameraConstants(String name, Transform3d robotToCamera, double stdDevFactor) {
      this(name, robotToCamera, stdDevFactor, VisionConstants.defaultFiducialFilter);
    }

    public CameraConstants(String name, Transform3d robotToCamera) {
      this(name, robotToCamera, 1.0);
    }
  }

  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  public static long[] defaultFiducialFilter = {
    2, 3, 4, 5, 8, 9, 10, 11, 13, 14, 15, 16, 18, 19, 20, 21, 24, 25, 26, 27, 29, 30, 31, 32
  };

  // Basic filtering thresholds
  public double maxAmbiguity = 0.3;
  public double maxZError = 0.75;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public double linearStdDevBaseline = 0.02; // Meters
  public double angularStdDevBaseline = 0.5; // Radians

  // Multipliers to apply for MegaTag 1 observations
  public double linearStdDevMegatag1Factor =
      Double.POSITIVE_INFINITY; // Don't use MT1 translation data in favor of MT2
  public double angularStdDevMegatag1Factor = 1.0; // Use MT1 rotation data

  // Multipliers to apply for MegaTag 2 observations
  public double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available
}
