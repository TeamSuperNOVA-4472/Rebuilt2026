package frc.robot.Subsystems;

import static frc.robot.Constants.SwerveConstants.kA;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.NetworkTableEvent.Kind;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class IntakeSubsystem extends SubsystemBase {
    public static final IntakeSubsystem kIntake = new IntakeSubsystem();

    public enum IntakeStorageMode{
        STORED,
        OUT
    }

    public enum IntakeActionMode{
        INTAKE,
        OUTTAKE,
        OFF
    }

    private IntakeStorageMode kStorageMode;
    private IntakeActionMode kActionMode;

    private boolean kIsAtState;
    private double kSliderTarget; 
    private double PIDOutput;

    private TalonFX kIntakeMotor;
    private TalonFX kIntakeSlider;
    private PIDController kSliderPID;

    private DCMotor kIntakeSimMotor;
    private ElevatorSim kIntakeSim;
    private Mechanism2d kSimSpace;
    private MechanismRoot2d kSimRoot;
    private MechanismLigament2d kSimDisp;

    private IntakeSubsystem(){
        kStorageMode = IntakeStorageMode.STORED;
        kActionMode = IntakeActionMode.OFF;
        kIsAtState = true;
        kIntakeMotor = new TalonFX(Constants.IntakeSubsystemConstants.kIntakeMotorPort);
        kIntakeSlider = new TalonFX(Constants.IntakeSubsystemConstants.kSliderMotorPort);
        kSliderPID = new PIDController(Constants.IntakeSubsystemConstants.kSliderP, Constants.IntakeSubsystemConstants.kSliderI, Constants.IntakeSubsystemConstants.kSliderD);
        kSliderPID.setTolerance(Constants.IntakeSubsystemConstants.kSlideThreshold, Constants.IntakeSubsystemConstants.kSlideSpeedThreshold);
        kIntakeSimMotor = DCMotor.getKrakenX44(1);
        kIntakeSim = new ElevatorSim(kIntakeSimMotor, Constants.IntakeSubsystemConstants.kGearing, Constants.IntakeSubsystemConstants.kMass, Constants.IntakeSubsystemConstants.kDrumRadius, 0, Constants.IntakeSubsystemConstants.kMaxLen, false, 0, 0, 0);
        kSimSpace = new Mechanism2d(60, 60);
        kSimRoot = kSimSpace.getRoot("base", 10, 30);
        kSimDisp = kSimRoot.append(new MechanismLigament2d("Intake", kIntakeSim.getPositionMeters()*Constants.IntakeSubsystemConstants.kSimLenMult, Constants.IntakeSubsystemConstants.kIntakeAngle));
        SmartDashboard.putData("IntakeSim", kSimSpace);

        TalonFXConfiguration kIntakeConfig = new TalonFXConfiguration();
        CurrentLimitsConfigs kIntakeCurrentConfig = new CurrentLimitsConfigs();
        MotorOutputConfigs kIntakeMotorConfig = new MotorOutputConfigs();
        kIntakeMotor.getConfigurator().refresh(kIntakeConfig);
        kIntakeMotor.getConfigurator().refresh(kIntakeCurrentConfig);
        kIntakeMotor.getConfigurator().refresh(kIntakeMotorConfig);
        kIntakeCurrentConfig.SupplyCurrentLimit = 10;
        kIntakeCurrentConfig.SupplyCurrentLimitEnable = true;
        kIntakeCurrentConfig.StatorCurrentLimitEnable = true;
        kIntakeCurrentConfig.StatorCurrentLimit = 10;
        kIntakeMotorConfig.NeutralMode = NeutralModeValue.Coast;
        kIntakeConfig.withCurrentLimits(kIntakeCurrentConfig);
        kIntakeConfig.withMotorOutput(kIntakeMotorConfig);
        kIntakeMotor.getConfigurator().apply(kIntakeConfig);

        TalonFXConfiguration kSliderConfig = new TalonFXConfiguration();
        CurrentLimitsConfigs kSliderCurrentConfig = new CurrentLimitsConfigs();
        MotorOutputConfigs kSliderMotorConfig = new MotorOutputConfigs();
        kIntakeSlider.getConfigurator().refresh(kSliderConfig);
        kIntakeSlider.getConfigurator().refresh(kSliderCurrentConfig);
        kIntakeSlider.getConfigurator().refresh(kSliderMotorConfig);
        kSliderCurrentConfig.SupplyCurrentLimit = 10;
        kSliderCurrentConfig.SupplyCurrentLimitEnable = true;
        kSliderCurrentConfig.StatorCurrentLimitEnable = true;
        kSliderCurrentConfig.StatorCurrentLimit = 10;
        kSliderMotorConfig.NeutralMode = NeutralModeValue.Coast;
        kSliderConfig.withCurrentLimits(kSliderCurrentConfig);
        kSliderConfig.withMotorOutput(kSliderMotorConfig);
        kIntakeSlider.getConfigurator().apply(kSliderConfig);
    }

    private void moveToStorageState(){
        switch (kStorageMode){
        case STORED:
            kActionMode = IntakeActionMode.OFF;
            kSliderTarget = Constants.IntakeSubsystemConstants.kStoredPos;
            break;

        case OUT:
            kSliderTarget = Constants.IntakeSubsystemConstants.kOutPos;
            break;
        
        }
    }

    private void setActionState(){
        switch (kActionMode){
        case INTAKE:
            kIntakeMotor.set(Constants.IntakeSubsystemConstants.kIntakeMotorSpeed);
            break;
        
        case OUTTAKE:
            kIntakeMotor.set(-Constants.IntakeSubsystemConstants.kIntakeMotorSpeed);
            break;
        
        case OFF:
            kIntakeMotor.set(0);
            break;
        }
    }
    
    public void setIntakeAction(IntakeActionMode mNewMode){
        if (kStorageMode.equals(IntakeStorageMode.OUT))
        {
            kActionMode = mNewMode;
            setActionState();
        }
    }

    public void setIntakeStorage(IntakeStorageMode mNewMode){
        kStorageMode = mNewMode;
        moveToStorageState();
    }

    public IntakeActionMode getActionMode(){
        return kActionMode;
    }

    public IntakeStorageMode getStorageMode(){
        return kStorageMode;
    }

    public boolean isReady(){
        return kIsAtState;
    }

    public double getIntakeSpeed(){
        return kIntakeMotor.get();
    }

    @Override
    public void periodic(){
        kIsAtState = kSliderPID.atSetpoint();
        if (Robot.isReal()){
            PIDOutput = MathUtil.clamp(kSliderPID.calculate(kIntakeSlider.getPosition().getValueAsDouble()*Constants.IntakeSubsystemConstants.kEncoderToInchesMult,kSliderTarget), -1, 1);
        } else{
            PIDOutput = MathUtil.clamp(kSliderPID.calculate(kIntakeSim.getPositionMeters()*39.3701,kSliderTarget), -1, 1);
        }
        kIntakeSlider.set(PIDOutput);
        SmartDashboard.putNumber("Current Action Mode: ", kActionMode.ordinal());
        SmartDashboard.putNumber("Current Storage Mode: ", kStorageMode.ordinal());
    }

    @Override
    public void simulationPeriodic(){
        kIntakeSim.setInput(PIDOutput * 12.0);

        kIntakeSim.update(0.02);

        kSimDisp.setLength(kIntakeSim.getPositionMeters()*Constants.IntakeSubsystemConstants.kSimLenMult);
        SmartDashboard.putNumber("Intake Length Horizontal", kIntakeSim.getPositionMeters()*39.3701*Math.cos(Math.toRadians(15)));
    }
}
