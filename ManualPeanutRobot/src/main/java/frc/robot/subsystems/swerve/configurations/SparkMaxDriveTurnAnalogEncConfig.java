package frc.robot.subsystems.swerve.configurations;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Constants;
import frc.robot.util.PIDConstants;

public class SparkMaxDriveTurnAnalogEncConfig implements ISwerveModuleConfiguration {

    public static final class Builder {
        private int mDriveCanId = 0;
        private int mTurnCanId = 0;
        private int mTurnAbsEncAnalogPort = 0;
        private double mTurnAbsEncOffset = 0.0;
        private PIDConstants mTurnPIDConstants = new PIDConstants(0, 0, 0, 0);
        private PIDConstants mDrivePIDConstants  = new PIDConstants(0, 0, 0, 0);

        public Builder withDriveCanId(int pDriveCanId) {
            mDriveCanId = pDriveCanId;
            return this;
        }

        public Builder withTurnCanId(int pTurnCanId) {
            mTurnCanId = pTurnCanId;
            return this;
        }

        public Builder withTurnAbsEncAnalogPortAndOffset(int pTurnAbsEncAnalogPort, double pTurnAbsEncOffset) {
            mTurnAbsEncAnalogPort = pTurnAbsEncAnalogPort;
            mTurnAbsEncOffset = pTurnAbsEncOffset;
            return this;
        }

        public Builder withTurnPIDConstants(PIDConstants pTurnPIDConstants) {
            mTurnPIDConstants = pTurnPIDConstants;
            return this;
        }

        public Builder withDrivePIDConstants(PIDConstants pDrivePIDConstants) {
            mDrivePIDConstants = pDrivePIDConstants;
            return this;
        }

        public SparkMaxDriveTurnAnalogEncConfig build() {
            return new SparkMaxDriveTurnAnalogEncConfig(this);
        }
    }

    private final SparkMax mDrive;
    private final SparkMax mTurn;
    private final AnalogPotentiometer mTurnEncAbs;
    private final PIDController mTurnPidController;
    private double mOffset = 0;
    private double mTargetDriveSpeed = 0.0;
    private double mTargetTurnAngle = 0.0;

    private SparkMaxDriveTurnAnalogEncConfig(Builder builder) {
        mDrive = new SparkMax(builder.mDriveCanId, MotorType.kBrushless);
        mTurn = new SparkMax(builder.mTurnCanId, MotorType.kBrushless);
        mTurnEncAbs = new AnalogPotentiometer(builder.mTurnAbsEncAnalogPort, 360, builder.mTurnAbsEncOffset);
        mTurnPidController = new PIDController(builder.mTurnPIDConstants.getP(),
                                               builder.mTurnPIDConstants.getI(),
                                               builder.mTurnPIDConstants.getD());
        SparkMaxConfig driveConfig = new SparkMaxConfig();
        driveConfig.voltageCompensation(12.0);
        driveConfig.smartCurrentLimit(40);

        SparkMaxConfig turnConfig = new SparkMaxConfig();
        turnConfig.voltageCompensation(12.0);
        turnConfig.smartCurrentLimit(30);
        mDrive.configure(driveConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        mTurn.configure(turnConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        mTurnPidController.enableContinuousInput(0, 360);
    }

    @Override
    public void setDriveSpeedMetersPerSecond(double pSpeed, boolean pIsOpenLoop) {
        mTargetDriveSpeed = pSpeed;
        mDrive.set(pSpeed / Constants.MAXIMUM_SPEED_METERS_PER_SECONDS);
    }

    @Override
    public void setTurnAngleDegrees(double pAngle) {
        double turnOutput = 0;
        if(Math.abs(mTargetDriveSpeed) >= 0.01 * Constants.MAXIMUM_SPEED_METERS_PER_SECONDS) { 
            mTargetTurnAngle = pAngle;
            turnOutput = mTurnPidController.calculate(getTurnAngleDegrees(), pAngle);
        }
        mTurn.set(turnOutput);
    }

    @Override
    public void recalibrateTurn() {
        // Do nothing since we are now relying on absolute encoder
    }

    @Override
    public void reset() {
        mDrive.getEncoder().setPosition(0);
        recalibrateTurn();
    } 

    @Override
    public double getDriveSpeedMetersPerSecond() {
        return mDrive.getEncoder().getVelocity() * Constants.DRIVE_DISTANCE_PER_NEO_ROTATION_METERS / 60;
    }

    @Override
    public double getTurnAngleDegrees() {
        return ((mTurnEncAbs.get() % 360) + 360) % 360;
    }

    @Override
    public double getTargetDriveSpeedMetersPerSecond() {
        return mTargetDriveSpeed;
    }

    @Override
    public double getTargetTurnAngleDegrees() {
        return mTargetTurnAngle;
    }

    @Override
    public double getOffset() {
        return mOffset;
    }

    @Override
    public double getDriveMeters() {
        return mDrive.getEncoder().getPosition() * Constants.DRIVE_DISTANCE_PER_NEO_ROTATION_METERS;
    }
}
