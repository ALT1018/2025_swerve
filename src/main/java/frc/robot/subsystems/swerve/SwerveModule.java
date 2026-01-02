package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule {
    private SparkMax m_rotorMotor;
    private TalonFX m_throttle;

    private VelocityVoltage throttleRequest = new VelocityVoltage(0);

    private CANcoder RotorCancoder;

    private PIDController rotorPID;

    public SwerveModule(int ThrottleID, int RotorID, int intRotorEncoderID, double RotorEncoderOffsetAngleDeg) {
        m_throttle = new TalonFX(ThrottleID);

        m_rotorMotor = new SparkMax(RotorID, MotorType.kBrushless);

        RotorCancoder = new CANcoder(intRotorEncoderID, "SwerveCancoder");

        TalonFXConfiguration throttleCfg = new TalonFXConfiguration();
        throttleCfg.MotorOutput.Inverted = SwerveConstants.kThrottleMotorInverted ? 
            InvertedValue.Clockwise_Positive : 
            InvertedValue.CounterClockwise_Positive;

        throttleCfg.CurrentLimits.SupplyCurrentLimit = SwerveConstants.kCurrentLimit;
        throttleCfg.CurrentLimits.SupplyCurrentLimitEnable = true;

        throttleCfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        throttleCfg.Slot0.kP = SwerveConstants.kDrive_P;
        throttleCfg.Slot0.kI = SwerveConstants.kDrive_I;
        throttleCfg.Slot0.kD = SwerveConstants.kDrive_D;
        throttleCfg.Slot0.kV = SwerveConstants.kDrive_kV;

        m_throttle.getConfigurator().apply(throttleCfg);

        m_rotorMotor.configure(
            new SparkMaxConfig()
                .inverted(SwerveConstants.kRotorMotorInverted)
                .idleMode(IdleMode.kBrake),
            ResetMode.kNoResetSafeParameters,
            PersistMode.kNoPersistParameters
        );
        
        //將上面設定的東西新增到Cancoder裡面
        RotorCancoder.getConfigurator().apply(
            new CANcoderConfiguration().MagnetSensor
                .withAbsoluteSensorDiscontinuityPoint(0.5)
                .withSensorDirection(SwerveConstants.kRotorEncoderdiretion)
                .withMagnetOffset(RotorEncoderOffsetAngleDeg/360)
        );

        //根據Constants裡設的常數設置Rotor PID
        rotorPID = new PIDController(
            SwerveConstants.kRotor_P,
            SwerveConstants.kRotor_I,
            SwerveConstants.kRotor_D
        );

        //啟用自動計算從當前值到目標值的最短路徑 180，-180視為同一個點不會嘗試轉一整圈
        rotorPID.enableContinuousInput(-180, 180);
    }
    
    //取得目前swerve模組得狀態(速度、旋轉角度)
    public SwerveModuleState getState() {
        //encoder.setVelocityConversionFactor(SwerveConstants.ThrottleVelocityConversionFactor);
        double driveRPS = m_throttle.getVelocity().getValueAsDouble();
        double ThrottleVelocity = driveRPS * SwerveConstants.kThrottleVelocityConversionFactor;

        return new SwerveModuleState(
            ThrottleVelocity,
            Rotation2d.fromRotations(RotorCancoder.getAbsolutePosition().getValueAsDouble())
        );
    }
    //取得目前swerve模組的狀態(位置、旋轉角度)
    public SwerveModulePosition getPosition() {
        //encoder.setPositionConversionFactor(SwerveConstants.ThrottlePositionConversionFactor);
        double ThrottlePosition = m_throttle.getPosition().getValueAsDouble() * SwerveConstants.kThrottlePositionConversionFactor;
        // double ThrottlePosition = -encoder.getPosition();

        return new SwerveModulePosition(
            ThrottlePosition,
            Rotation2d.fromRotations(RotorCancoder.getAbsolutePosition().getValueAsDouble())
        );  
    }
    
    //設定Swerve模組如何運作
    public void setState(SwerveModuleState state) {
        // 優化狀態，使轉向馬達不必旋轉超過 90 度來獲得目標的角度
        state.optimize(this.getState().angle);

        //比較目前角度與目標角度利用PID控制器計算出馬達需要輸出多少
        double rotorOutput = rotorPID.calculate(getState().angle.getDegrees(), state.angle.getDegrees());
        m_rotorMotor.set(rotorOutput);

        double targetRPS = state.speedMetersPerSecond * SwerveConstants.kThrottleVelocityConversionFactor;
        m_throttle.setControl(throttleRequest.withVelocity(targetRPS));
    }

    public void setStateVoltage(SwerveModuleState state) {
        // 優化狀態，使轉向馬達不必旋轉超過 90 度來獲得目標的角度
        state.optimize(this.getState().angle);

        //比較目前角度與目標角度利用PID控制器計算出馬達需要輸出多少
        double rotorOutput = rotorPID.calculate(getState().angle.getDegrees(), state.angle.getDegrees());
        m_rotorMotor.set(rotorOutput);

        m_throttle.set(state.speedMetersPerSecond);
    }

    public void setRotorangle() {
        double rotorOutput = rotorPID.calculate(getState().angle.getDegrees(), 90);
        m_rotorMotor.set(rotorOutput);
    }

    public double get() {
        return m_throttle.get();
    }
}
