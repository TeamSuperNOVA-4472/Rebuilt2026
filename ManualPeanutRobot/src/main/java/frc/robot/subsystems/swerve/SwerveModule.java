package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.swerve.configurations.ISwerveModuleConfiguration;

public class SwerveModule {

    private final String mName;

    private final ISwerveModuleConfiguration mConfiguration;

    public SwerveModule(ISwerveModuleConfiguration pConfiguration) {
        this("", pConfiguration);
    }

    public SwerveModule(String pName, ISwerveModuleConfiguration pConfiguration) {
        mName = pName;
        mConfiguration = pConfiguration;
    }
    
    public void setState(SwerveModuleState pModuleState, boolean pIsOpenLoop) {
        pModuleState.optimize(Rotation2d.fromDegrees(mConfiguration.getTurnAngleDegrees()));
        mConfiguration.setDriveSpeedMetersPerSecond(pModuleState.speedMetersPerSecond, pIsOpenLoop);
        mConfiguration.setTurnAngleDegrees(pModuleState.angle.getDegrees());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(mConfiguration.getDriveSpeedMetersPerSecond(),
                                    Rotation2d.fromDegrees(mConfiguration.getTurnAngleDegrees()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(mConfiguration.getDriveMeters(),Rotation2d.fromDegrees(mConfiguration.getTurnAngleDegrees()));
    }

    public void reset() {
        mConfiguration.reset();
    }
    
    public void recalibrateTurn() {
        mConfiguration.recalibrateTurn();
    }

    public double getOffset() {
        return mConfiguration.getOffset();
    }

    public void outputDebug() {
        SmartDashboard.putNumber(mName + " Speed (m/s)", mConfiguration.getDriveSpeedMetersPerSecond());
        SmartDashboard.putNumber(mName + " Turn (Deg)", mConfiguration.getTurnAngleDegrees());
        SmartDashboard.putNumber(mName + " Target Speed (m/s)", mConfiguration.getTargetDriveSpeedMetersPerSecond());
        SmartDashboard.putNumber(mName + " Target Turn (Deg)", mConfiguration.getTargetTurnAngleDegrees());
        SmartDashboard.putNumber(mName + " Offset (Deg)", getOffset());
    }
}
