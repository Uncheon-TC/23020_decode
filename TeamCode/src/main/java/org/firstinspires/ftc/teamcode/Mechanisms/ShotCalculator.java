package org.firstinspires.ftc.teamcode.Mechanisms;

public class ShotCalculator {
    public static final double GRAVITY = 386.1;

    public static class ShotResult {
        public final double hoodAngle;
        public final double launchSpeed;
        public final double turretOffset;
        public final double timeOfFlight;
        public final double distanceToGoal;

        public ShotResult(double hoodAngle, double launchSpeed, double turretOffset,
                          double timeOfFlight, double distanceToGoal) {
            this.hoodAngle = hoodAngle;
            this.launchSpeed = launchSpeed;
            this.turretOffset = turretOffset;
            this.timeOfFlight = timeOfFlight;
            this.distanceToGoal = distanceToGoal;
        }
    }

    public static ShotResult calculateShot(double robotX, double robotY, double goalX, double goalY,
                                           double goalHeight, double robotVelX, double robotVelY,
                                           double targetEntryAngle) {
        double dx = goalX - robotX;
        double dy = goalY - robotY;
        double distToGoal = Math.hypot(dx, dy);
        if (distToGoal <= 0) {
            return null;
        }

        double angleToGoal = Math.atan2(dy, dx);
        double alphaStatic = Math.atan((2 * goalHeight) / distToGoal - Math.tan(targetEntryAngle));

        double cosAlpha = Math.cos(alphaStatic);
        double tanAlpha = Math.tan(alphaStatic);
        double denominator = 2 * Math.pow(cosAlpha, 2) * ((distToGoal * tanAlpha) - goalHeight);
        if (denominator <= 0) {
            return null;
        }

        double v0Static = Math.sqrt(GRAVITY * Math.pow(distToGoal, 2) / denominator);
        double timeOfFlight = distToGoal / (v0Static * cosAlpha);

        double robotVelMag = Math.hypot(robotVelX, robotVelY);
        double robotVelAngle = Math.atan2(robotVelY, robotVelX);
        double thetaDiff = robotVelAngle - angleToGoal;
        double radialVelocity = -Math.cos(thetaDiff) * robotVelMag;
        double tangentialVelocity = Math.sin(thetaDiff) * robotVelMag;

        double compensatedHorizontal = (distToGoal / timeOfFlight) + radialVelocity;
        double newHorizontal = Math.sqrt(Math.pow(compensatedHorizontal, 2)
                + Math.pow(tangentialVelocity, 2));
        double verticalVelocity = v0Static * Math.sin(alphaStatic);
        double hoodAngle = Math.atan2(verticalVelocity, newHorizontal);

        double adjustedDistance = newHorizontal * timeOfFlight;
        double cosNew = Math.cos(hoodAngle);
        double tanNew = Math.tan(hoodAngle);
        double newDenominator = 2 * Math.pow(cosNew, 2)
                * ((adjustedDistance * tanNew) - goalHeight);
        double launchSpeed = newDenominator > 0
                ? Math.sqrt(GRAVITY * Math.pow(adjustedDistance, 2) / newDenominator)
                : v0Static;

        double turretOffset = Math.atan2(tangentialVelocity, compensatedHorizontal);
        return new ShotResult(hoodAngle, launchSpeed, turretOffset, timeOfFlight, distToGoal);
    }
}
