package frc.robot.Subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.TurretConstants;

public class TurretSubsystem extends SubsystemBase 
{
    private final TalonFX kTurretMotor;

    private final PIDController kPidController;
    private final SimpleMotorFeedforward kSpinningFeedForward;
    private Supplier<Double> kGetAngularVelocity;

    private double kTurretTargetAngle = 0.0;
    private double kOutput;
    private boolean kPIDEnabled = true;
    private boolean kIsSafeModeEnabled = false;

    private final DCMotor kTurretSimMotor;
    private final SingleJointedArmSim kTurretSim;
    private final Mechanism2d kSimSpace;
    private final MechanismRoot2d kSimRoot;
    private final MechanismLigament2d kSimDisp;

    public TurretSubsystem() 
    {
        kTurretMotor = new TalonFX(TurretConstants.kTurretMotorPort, TurretConstants.kTurretCanbus);

        kPidController = new PIDController(TurretConstants.kTurretP, TurretConstants.kTurretI, TurretConstants.kTurretD);
        kSpinningFeedForward = new SimpleMotorFeedforward(0, TurretConstants.kTurretV, TurretConstants.kTurretA); // ks is manually adjusted for sign
        kTurretSimMotor = DCMotor.getKrakenX44(TurretConstants.kSimNumMotor);
        kTurretSim = new SingleJointedArmSim(kTurretSimMotor, TurretConstants.kGearing, TurretConstants.kSimjKgMetersSquared, TurretConstants.kSimArmLength, 0, TurretConstants.kDeadband * Math.PI / 180.0, false, 0, 0, 0);
        kSimSpace = new Mechanism2d(TurretConstants.kSimWidth, TurretConstants.kSimHeight);
        kSimRoot = kSimSpace.getRoot(TurretConstants.kSimRootName, TurretConstants.kSimX, TurretConstants.kSimY);
        kSimDisp = kSimRoot.append(new MechanismLigament2d("Turret",TurretConstants.kSimLength, kTurretSim.getAngleRads() * 180 / Math.PI));
        SmartDashboard.putData("TurretSim", kSimSpace);

        TalonFXConfiguration kTurretConfig = new TalonFXConfiguration();
        CurrentLimitsConfigs kTurretCurrentConfig = new CurrentLimitsConfigs();
        MotorOutputConfigs kTurretMotorConfig = new MotorOutputConfigs();
        kTurretMotor.getConfigurator().refresh(kTurretConfig);
        kTurretMotor.getConfigurator().refresh(kTurretCurrentConfig);
        kTurretMotor.getConfigurator().refresh(kTurretMotorConfig);
        kTurretCurrentConfig.SupplyCurrentLimit = TurretConstants.kTurretSupplyLimit;
        kTurretCurrentConfig.SupplyCurrentLimitEnable = TurretConstants.kTurretSupplyLimitEnabled;
        kTurretCurrentConfig.StatorCurrentLimitEnable = TurretConstants.kTurretStatorLimitEnabled;
        kTurretCurrentConfig.StatorCurrentLimit = TurretConstants.kTurretStatorLimit;
        kTurretMotorConfig.NeutralMode = TurretConstants.kTurretNeutralMode;
        kTurretConfig.withCurrentLimits(kTurretCurrentConfig);
        kTurretConfig.withMotorOutput(kTurretMotorConfig);
        kTurretMotor.getConfigurator().apply(kTurretConfig);

        kPidController.setTolerance(TurretConstants.kTurretTolerance);
        kTurretMotor.setPosition(0);
        kGetAngularVelocity = () -> 0.0;
    }

    public void setAngularSpeedSupplier(Supplier<Double> speed)
    {
        kGetAngularVelocity = speed;
    }

    public Boolean getSafeModeEnabled()
    {
        return kIsSafeModeEnabled;
    }

    public void enableSafeMode()
    {
        kIsSafeModeEnabled = true;
    }
    
    public void disableSafeMode()
    {
        kIsSafeModeEnabled = false;
    }

    public void rotate(double speed) 
    {
        kTurretMotor.set(speed);
    }

    public void stop() 
    {
        kTurretMotor.stopMotor();
    }

    public boolean getTurretAtSetpoint()
    {
        return kPidController.atSetpoint() && isValidAngle();
    }

    public void disablePID()
    {
        kPIDEnabled = false;
    }

    public void enablePID()
    {
        kPIDEnabled = true;
    }

    public double getAngle() 
    {
        double encoderPosition = kTurretMotor.getPosition().getValueAsDouble();

        return (encoderPosition * TurretConstants.kGearing) * 360;
    }

    public double getStator()
    {
        return kTurretMotor.getStatorCurrent().getValueAsDouble();
    }

    public void resetEncoder()
    {
        kTurretMotor.setPosition(0);
    }

    private void goToAngle(double targetAngle) 
    {
        double currentAngle;
        if (Robot.isReal()) currentAngle = getAngle();
        else currentAngle = kTurretSim.getAngleRads()*180/Math.PI;

        kOutput = MathUtil.clamp(kSpinningFeedForward.calculate(kGetAngularVelocity.get()) + kPidController.calculate(currentAngle, targetAngle), -TurretConstants.kMaxSpeedOutput, TurretConstants.kMaxSpeedOutput); // kSpinningFeedForward.calculate(kGetAngularVelocity.get())
        double addition = kOutput >= 0 ? TurretConstants.kTurretS : -TurretConstants.kTurretS;
        kTurretMotor.set(kOutput + addition);
    }

    public boolean isValidAngle() {
        if (kTurretTargetAngle >= TurretConstants.kDeadband) {
            return false;
        } else {
            return true;
        }  
    }

    public void setTargetAngle(double mNewAngle) {
        kTurretTargetAngle = mNewAngle;
    }

    @Override
    public void periodic() {
        if (kPIDEnabled)
        {
            if (isValidAngle()) {
                goToAngle(kTurretTargetAngle);
            } else {
                goToAngle(TurretConstants.kDeadband);
            }
        }

        SmartDashboard.putNumber("Subsystems/TurretSubsystem/Relative Angle: ", getAngle());
        SmartDashboard.putNumber("Subsystems/TurretSubsystem/Goal Angle: ", kTurretTargetAngle);
    
    }

    @Override
    public void simulationPeriodic() {
      kTurretSim.setInput(kOutput * TurretConstants.kSimMultiplier);

      kTurretSim.update(TurretConstants.kSimdt);

      kSimDisp.setAngle(kTurretSim.getAngleRads()*180 / Math.PI);
    }
}