package frc.robot.subsystems.swerve.configurations;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Constants;
import frc.robot.util.PIDConstants;

public class TalonDriveSparkTurnAnalogEncConfig implements ISwerveModuleConfiguration {

    public static final class Builder {
        private int mDriveCanId = 0;
        private int mTurnPwmPort = 0;
        private int mTurnAbsEncAnalogPort = 0;
        private double mTurnAbsEncOffset = 0.0;
        private PIDConstants mTurnPIDConstants = new PIDConstants(0, 0, 0, 0);
        private PIDConstants mDrivePIDConstants  = new PIDConstants(0, 0, 0, 0);

        public Builder withDriveCanId(int pDriveCanId) {
            mDriveCanId = pDriveCanId;
            return this;
        }

        public Builder withTurnPwmPort(int pTurnPwmPort) {
            mTurnPwmPort = pTurnPwmPort;
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

        public TalonDriveSparkTurnAnalogEncConfig build() {
            return new TalonDriveSparkTurnAnalogEncConfig(this);
        }
    }

    private final WPI_TalonSRX mDrive;
    private final MotorController mTurn;
    private final AnalogPotentiometer mTurnEncAbs;
    private final PIDController mTurnPidController;
    private final SimpleMotorFeedforward mFeedForward = new SimpleMotorFeedforward(Constants.DRIVE_KS, Constants.DRIVE_KV, Constants.DRIVE_KA);
    private double mOffset = 0;
    private double mTargetDriveSpeed = 0.0;
    private double mTargetTurnAngle = 0.0;

    private TalonDriveSparkTurnAnalogEncConfig(Builder builder) {
        mDrive = new WPI_TalonSRX(builder.mDriveCanId);
        mTurn = new Spark(builder.mTurnPwmPort);
        mTurnEncAbs = new AnalogPotentiometer(builder.mTurnAbsEncAnalogPort, 360, builder.mTurnAbsEncOffset);
        mTurnPidController = new PIDController(builder.mTurnPIDConstants.getP(),
                                               builder.mTurnPIDConstants.getI(),
                                               builder.mTurnPIDConstants.getD());
        mDrive.config_kP(0, builder.mDrivePIDConstants.getP());
        mDrive.config_kI(0, builder.mDrivePIDConstants.getI());
        mDrive.config_kD(0, builder.mDrivePIDConstants.getD());
        mDrive.config_kF(0, builder.mDrivePIDConstants.getF());
        mDrive.config_kF(0, 0);
        mDrive.configVoltageCompSaturation(10);
        mDrive.configPeakCurrentLimit(0);
        mDrive.configPeakCurrentDuration(0);
        mDrive.configContinuousCurrentLimit(30);
        mDrive.enableCurrentLimit(true);
        mDrive.enableVoltageCompensation(true);
        mDrive.setSensorPhase(true);
        mDrive.selectProfileSlot(0, 0);
        mTurnPidController.enableContinuousInput(0, 360);
    }

    @Override
    public void setDriveSpeedMetersPerSecond(double pSpeed, boolean pIsOpenLoop) {
        mTargetDriveSpeed = pSpeed;
        if(pIsOpenLoop) {
            mDrive.set(ControlMode.PercentOutput, pSpeed / Constants.MAXIMUM_SPEED_METERS_PER_SECONDS);
        } else {
            double driveSpeedTics = pSpeed * Constants.METERS_PER_SECOND_TO_DRIVE_SPEED_TICS;
            mDrive.set(ControlMode.Velocity, driveSpeedTics, DemandType.ArbitraryFeedForward, mFeedForward.calculate(pSpeed));
        }
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
        mDrive.setSelectedSensorPosition(0);
        recalibrateTurn();
    } 

    @Override
    public double getDriveSpeedMetersPerSecond() {
        return mDrive.getSelectedSensorVelocity() * Constants.DRIVE_DISTANCE_PER_TIC_METERS * 10;
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
        return mDrive.getSelectedSensorPosition() * Constants.DRIVE_DISTANCE_PER_TIC_METERS;
    }
}
