package frc.robot;

public class SpeedLimit {
    private double speedLimit;

    public SpeedLimit(double speedLimit) {
        this.speedLimit = speedLimit;
    }

    public double apply(double speed) {
        return Math.min(Math.abs(speed), speedLimit) * Math.signum(speed);
    }
}
