package frc.robot.Subsystems;

import java.lang.annotation.Target;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class TurretSubsystem extends SubsystemBase 
{
    private final TalonFX kTurretMotor;

    private static final double kRevolutions = 2048;

    private static final double kP = 0.013;

    private static final double kI = 0.0;

    private static final double kD = 0.0008;

    private final PIDController kPidController;

    private static final double kDeadband = 330.0;

    private double kTurretTargetAngle = 0.0;

    private DCMotor kTurretSimMotor;
    private SingleJointedArmSim kTurretSim;
    private Mechanism2d kSimSpace;
    private MechanismRoot2d kSimRoot;
    private MechanismLigament2d kSimDisp;
    private double kOutput;
    public static TurretSubsystem kTurret = new TurretSubsystem();

    private TurretSubsystem() 
    {
        kTurretMotor = new TalonFX(24);

        kPidController = new PIDController(kP, kI, kD);

        kTurretSimMotor = DCMotor.getKrakenX44(1);
        kTurretSim = new SingleJointedArmSim(kTurretSimMotor, 24.668, 0.291, 0.2921, 0, kDeadband * Math.PI / 180.0, false, 0, 0, 0);
        kSimSpace = new Mechanism2d(60, 60);
        kSimRoot = kSimSpace.getRoot("base", 30, 30);
        kSimDisp = kSimRoot.append(new MechanismLigament2d("Turret",10 , kTurretSim.getAngleRads() * 180 / Math.PI));
        SmartDashboard.putData("TurretSim", kSimSpace);

        TalonFXConfiguration kTurretConfig = new TalonFXConfiguration();
        CurrentLimitsConfigs kTurretCurrentConfig = new CurrentLimitsConfigs();
        MotorOutputConfigs kTurretMotorConfig = new MotorOutputConfigs();
        kTurretMotor.getConfigurator().refresh(kTurretConfig);
        kTurretMotor.getConfigurator().refresh(kTurretCurrentConfig);
        kTurretMotor.getConfigurator().refresh(kTurretMotorConfig);
        kTurretCurrentConfig.SupplyCurrentLimit = 10;
        kTurretCurrentConfig.SupplyCurrentLimitEnable = true;
        kTurretCurrentConfig.StatorCurrentLimitEnable = true;
        kTurretCurrentConfig.StatorCurrentLimit = 10;
        kTurretMotorConfig.NeutralMode = NeutralModeValue.Coast;
        kTurretConfig.withCurrentLimits(kTurretCurrentConfig);
        kTurretConfig.withMotorOutput(kTurretMotorConfig);
        kTurretMotor.getConfigurator().apply(kTurretConfig);
    }

    public void rotate(double speed) 
    {
        kTurretMotor.set(speed);
    }

    public void stop() 
    {
        kTurretMotor.stopMotor();
    }

    public double getAngle() 
    {
        double encoderPosition = kTurretMotor.getPosition().getValueAsDouble();

        return (encoderPosition / kRevolutions) * 360;
    }

    public void goToAngle(double targetAngle) 
    {
        double currentAngle;
        if (Robot.isReal()) currentAngle = getAngle();
        else currentAngle = kTurretSim.getAngleRads()*180/Math.PI;

        kOutput = MathUtil.clamp(kPidController.calculate(currentAngle, targetAngle), -1, 1);

        kTurretMotor.set(kOutput);
    }

    public boolean isValidAngle() {
        if (kTurretTargetAngle >= kDeadband) {
            return false;
        } else{
            return true;
        }  
    }
     public void setTargetAngle(double mNewAngle) {
        kTurretTargetAngle = mNewAngle;
    }

    @Override
    public void periodic() {
        if (isValidAngle()) {
            goToAngle(kTurretTargetAngle);
        }
    }

    @Override
    public void simulationPeriodic() {
      kTurretSim.setInput(kOutput * 12.0);

      kTurretSim.update(0.02);

      kSimDisp.setAngle(kTurretSim.getAngleRads()*180 / Math.PI);
      SmartDashboard.putNumber("Turret Angle", kTurretSim.getAngleRads()*180 / Math.PI);
      SmartDashboard.putNumber("Turret Target", kTurretTargetAngle);
    }
}