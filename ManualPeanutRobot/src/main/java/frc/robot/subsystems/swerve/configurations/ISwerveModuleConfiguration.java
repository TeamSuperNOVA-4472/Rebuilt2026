package frc.robot.subsystems.swerve.configurations;

public interface ISwerveModuleConfiguration {
    void setDriveSpeedMetersPerSecond(double pSpeed, boolean pIsOpenLoop);
    void setTurnAngleDegrees(double pAngle);
    void recalibrateTurn();
    void reset();
    double getDriveSpeedMetersPerSecond();
    double getTurnAngleDegrees();
    double getDriveMeters();
    double getTargetDriveSpeedMetersPerSecond();
    double getTargetTurnAngleDegrees();
    double getOffset();
}
