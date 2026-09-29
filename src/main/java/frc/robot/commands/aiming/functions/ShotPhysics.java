package frc.robot.commands.aiming.functions;

/**
 * Plain-Java (no WPILib) ballistics for a hub shot. Given the horizontal distance from the shooter
 * to the target, it picks a launch angle that makes the ball drop into the hub at a chosen entry
 * angle, then works out the exit speed, flywheel RPM, hood angle and time of flight.
 *
 * <p>Model: gravity only (no drag). Drag, spin and ball compression are absorbed into {@link
 * Constants#exitSpeedRatio()}, which was fitted to the measured shot tables. Units: meters,
 * seconds, degrees, RPM. Angles are measured from horizontal unless the name says "hood".
 */
public final class ShotPhysics {
  private static final double GRAVITY = 9.81; // m/s^2

  /** Distances outside this range are clamped before solving. */
  private static final double MIN_DISTANCE_METERS = 0.5;

  private static final double MAX_DISTANCE_METERS = 7.5;

  public record Constants(
      /** Height of the ball when it leaves the shooter. */
      double exitHeightMeters,
      /** Height of the point being aimed at (hub opening plane). */
      double targetHeightMeters,
      double wheelDiameterMeters,
      /** Ball exit speed divided by flywheel surface speed (about 0.44 on this robot). */
      double exitSpeedRatio,
      /** Preferred angle the ball descends at when it reaches the hub, below horizontal. */
      double desiredEntryDeg,
      /** The steepest entry angle the solver may fall back to if the hood cannot reach. */
      double maxEntryDeg,
      /** Fit from measured shots: launchAngleDeg = intercept + slope * hoodAngleDeg. */
      double hoodFitInterceptDeg,
      double hoodFitSlope,
      double minHoodDeg,
      double maxHoodDeg,
      double maxRpm) {}

  public record Result(
      double hoodDeg,
      double rpm,
      double timeOfFlightSecs,
      double launchDeg,
      double entryDeg) {}

  private final Constants c;

  public ShotPhysics(Constants constants) {
    this.c = constants;
  }

  /** Launch angle (deg above horizontal) so the ball arrives at the target at entryDeg. */
  private static double launchForEntry(double distance, double heightDiff, double entryDeg) {
    // For a parabola hitting (distance, heightDiff):
    // tan(entry) = tan(launch) - 2 * heightDiff / distance (entry positive = descending).
    double tanLaunch = Math.tan(Math.toRadians(entryDeg)) + 2.0 * heightDiff / distance;
    return Math.toDegrees(Math.atan(tanLaunch));
  }

  public Result solve(double distanceMeters) {
    double d = Math.max(MIN_DISTANCE_METERS, Math.min(MAX_DISTANCE_METERS, distanceMeters));
    double heightDiff = c.targetHeightMeters() - c.exitHeightMeters();

    // Start at the preferred entry angle. If the hood would have to go past its maximum (a flatter
    // shot than it can make), use a steeper entry until the hood can reach it.
    double entry = c.desiredEntryDeg();
    double launch = launchForEntry(d, heightDiff, entry);
    double hood = (launch - c.hoodFitInterceptDeg()) / c.hoodFitSlope();
    while (hood > c.maxHoodDeg() && entry < c.maxEntryDeg()) {
      entry += 1.0;
      launch = launchForEntry(d, heightDiff, entry);
      hood = (launch - c.hoodFitInterceptDeg()) / c.hoodFitSlope();
    }

    // Limit to the hood's real travel, then recompute the launch angle the hood will actually make.
    hood = Math.max(c.minHoodDeg(), Math.min(c.maxHoodDeg(), hood));
    launch = c.hoodFitInterceptDeg() + c.hoodFitSlope() * hood;

    double launchRad = Math.toRadians(launch);
    double cosLaunch = Math.cos(launchRad);
    double tanLaunch = Math.tan(launchRad);

    // Speed needed to hit (d, heightDiff) at this launch angle. The denominator is positive as long
    // as the launch is steep enough to reach the target; keep it from reaching zero.
    double denom = Math.max(0.05, d * tanLaunch - heightDiff);
    double speed = Math.sqrt(GRAVITY * d * d / (2.0 * cosLaunch * cosLaunch * denom));
    double timeOfFlight = d / (speed * cosLaunch);
    double actualEntry = Math.toDegrees(Math.atan(tanLaunch - 2.0 * heightDiff / d));

    double surfaceSpeed = speed / c.exitSpeedRatio();
    double rpm = surfaceSpeed / (Math.PI * c.wheelDiameterMeters()) * 60.0;
    rpm = Math.min(rpm, c.maxRpm());

    return new Result(hood, rpm, timeOfFlight, launch, actualEntry);
  }
}
