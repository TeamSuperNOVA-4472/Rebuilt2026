package frc.robot.Subsystems;

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

    public enum IntakeMode{
        STORED,
        INTAKE,
        OUTTAKE
    }
    private IntakeMode kMode;

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
        kMode = IntakeMode.STORED;
        kIsAtState = true;
        kIntakeMotor = new TalonFX(Constants.IntakeSubsystemConstants.kIntakeMotorPort);
        kIntakeSlider = new TalonFX(Constants.IntakeSubsystemConstants.kSliderMotorPort);
        kSliderPID = new PIDController(Constants.IntakeSubsystemConstants.kSliderP, Constants.IntakeSubsystemConstants.kSliderI, Constants.IntakeSubsystemConstants.kSliderD);
        kSliderPID.setTolerance(Constants.IntakeSubsystemConstants.kSlideThreshold, Constants.IntakeSubsystemConstants.kSlideSpeedThreshold);
        kIntakeSimMotor = DCMotor.getKrakenX60(1);
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

    private void moveToState(){
        switch (kMode){
        case STORED:
            kSliderTarget = Constants.IntakeSubsystemConstants.kStoredPos;
            kIntakeMotor.set(0);
            break;
        
        case INTAKE:
            kSliderTarget = Constants.IntakeSubsystemConstants.kOutPos;
            kIntakeMotor.set(Constants.IntakeSubsystemConstants.kIntakeMotorSpeed);
            break;
        
        case OUTTAKE:
            kSliderTarget = Constants.IntakeSubsystemConstants.kOutPos;
            kIntakeMotor.set(-Constants.IntakeSubsystemConstants.kIntakeMotorSpeed);
            break;
        
        }
    }
    
    public void setIntake(IntakeMode mNewMode){
        kMode = mNewMode;
        moveToState();
    }
    public IntakeMode getMode(){
        return kMode;
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
            //TODO: Fix kEncoderToInchesMult BEFORE TESTING
            PIDOutput = MathUtil.clamp(kSliderPID.calculate(kIntakeSlider.getPosition().getValueAsDouble()*Constants.IntakeSubsystemConstants.kEncoderToInchesMult,kSliderTarget), -1, 1);
        } else{
            PIDOutput = MathUtil.clamp(kSliderPID.calculate(kIntakeSim.getPositionMeters()*39.3701,kSliderTarget), -1, 1);
        }
        kIntakeSlider.set(PIDOutput);
        SmartDashboard.putNumber("Current Mode", kMode.ordinal());
    }

    @Override
    public void simulationPeriodic(){
        kIntakeSim.setInput(PIDOutput * 12.0);

        kIntakeSim.update(0.02);

        kSimDisp.setLength(kIntakeSim.getPositionMeters()*Constants.IntakeSubsystemConstants.kSimLenMult);
        SmartDashboard.putNumber("Intake Length Horizontal", kIntakeSim.getPositionMeters()*39.3701*Math.cos(Math.toRadians(15)));
    }
}
