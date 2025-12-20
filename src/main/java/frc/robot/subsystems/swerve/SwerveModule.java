package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule {
    private SparkMax m_rotorMotor;
    private SparkMax m_throttle;

    private RelativeEncoder encoder;
    private CANcoder RotorCancoder;

    private PIDController rotorPID;

    private static final SimpleMotorFeedforward ff_throttleMotor = new SimpleMotorFeedforward(0, SwerveConstants.kThottleFF_kV, SwerveConstants.kThottleFF_kA);

    public SwerveModule(int ThrottleID, int RotorID, int intRotorEncoderID, double RotorEncoderOffsetAngleDeg) {
        m_throttle = new SparkMax(ThrottleID, MotorType.kBrushless);

        m_rotorMotor = new SparkMax(RotorID, MotorType.kBrushless);

        RotorCancoder = new CANcoder(intRotorEncoderID, "SwerveCancoder");

        m_throttle.configure(
            new SparkMaxConfig()
                .inverted(SwerveConstants.kThrottleMotorInverted)
                .idleMode(IdleMode.kBrake)
                .apply(new EncoderConfig()
                    .velocityConversionFactor(SwerveConstants.ThrottleVelocityConversionFactor)
                    .positionConversionFactor(SwerveConstants.ThrottlePositionConversionFactor)
                ),
            ResetMode.kNoResetSafeParameters,
            PersistMode.kNoPersistParameters
        );
        m_rotorMotor.configure(
            new SparkMaxConfig()
                .inverted(SwerveConstants.kRotorMotorInverted)
                .idleMode(IdleMode.kBrake),
            ResetMode.kNoResetSafeParameters,
            PersistMode.kNoPersistParameters
        );

        encoder = m_throttle.getEncoder();
        
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
        double ThrottleVelocity = encoder.getVelocity();

        return new SwerveModuleState(
            ThrottleVelocity,
            Rotation2d.fromRotations(RotorCancoder.getAbsolutePosition().getValueAsDouble())
        );
    }
    //取得目前swerve模組的狀態(位置、旋轉角度)
    public SwerveModulePosition getPosition() {
        //encoder.setPositionConversionFactor(SwerveConstants.ThrottlePositionConversionFactor);
        double ThrottlePosition = -encoder.getPosition();

        return new SwerveModulePosition(
            ThrottlePosition,
            Rotation2d.fromRotations(RotorCancoder.getAbsolutePosition().getValueAsDouble())
        );  
    }
    
    //設定Swerve模組如何運作
    public void setState(SwerveModuleState state, boolean isffControl) {
        state.optimize(this.getState().angle);

        //比較目前角度與目標角度利用PID控制器計算出馬達需要輸出多少
        double rotorOutput = rotorPID.calculate(getState().angle.getDegrees(), state.angle.getDegrees());

        if(isffControl) {  
            m_throttle.setVoltage(ff_throttleMotor.calculateWithVelocities(this.getState().speedMetersPerSecond, state.speedMetersPerSecond));
        } else {
            m_throttle.set(state.speedMetersPerSecond);
        }
        m_rotorMotor.set(rotorOutput);
    
    }

    public void setVoltage(Voltage voltage) {
        m_throttle.setVoltage(voltage);
    }

    public void setRotorangle() {
        double rotorOutput = rotorPID.calculate(getState().angle.getDegrees(), 90);
        m_rotorMotor.set(rotorOutput);
    }

    public double get() {
        return m_throttle.get();
    }
}
