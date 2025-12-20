// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;

public final class Constants {
	public final class SwerveConstants {
		//轉向馬達ID
		public static final int kRightFrontRotorID = 53;
		public static final int kRightRearRotorID = 55;
		public static final int kLeftFrontRotorID = 33;
		public static final int kLeftRearRotorID = 34;

		//動力馬達ID
		public static final int kRightFrontThrottleID = 43;
		public static final int kRightRearThrottleID = 30;
		public static final int kLeftFrontThrottleID = 57;
		public static final int kLeftRearThrottleID = 59;

		//轉向馬達EncoderID
		public static final int kRightFrontEncoderID = 2;
		public static final int kRightRearEncoderID = 3;
		public static final int kLeftFrontEncoderID = 1;
		public static final int kLeftRearEncoderID = 4;

		//轉向馬達encoder以及轉向馬達方向設置
		public static final SensorDirectionValue kRotorEncoderdiretion = SensorDirectionValue.CounterClockwise_Positive;

		//Rotor Encode 偏移量    
		public static final double kLeftFrontRotorEncoderOffset = -41.9238 - 0.1757 + 0.6152 + 6.4  + 0.6 - 0.08 - 0.26 + 0.43  +3.867 -4.3 + 1.14 + 0.878;
		public static final double kLeftRearRotorEncoderOffset = -82.001953125 - 1.054;
		public static final double kRightFrontRotorEncoderOffset = -39.9023 + 180 + 0.175 - 1.58 + 0.79 - 0.087 + 1.83 - 0.2636 + 0.52 - 2.636 - 1.31;
		public static final double kRightRearRotorEncoderOffset = -40.166 + 0.966 + 2.2 - 4.5 + 0.6 - 0.36 + 3.16 -2.37 - 0.26 + 0.43 + 0.7 - 0.088 - 0.351;

		public static final double kwhatever = 0.60325;//meter

		public static final double khowlongismyrobot = kwhatever + 0.2;

		//swerve kinematics 四輪位置順序(LF,RF,LR,RR)
		public static final SwerveDriveKinematics kSwerveDriveKinematics = new SwerveDriveKinematics(
			 new Translation2d(kwhatever/2, kwhatever/2),
			 new Translation2d(kwhatever/2, kwhatever/-2),
			 new Translation2d(kwhatever/-2, kwhatever/2),
			 new Translation2d(kwhatever/-2, kwhatever/-2)
		);

		//Rotor PID constants(還沒搞懂要幹嘛)
		public static final double kRotor_P = 0.008181818182;//0.00585;
		public static final double kRotor_I = 0.0001374545455;//0.00001;
		public static final double kRotor_D = 0;//0.000058;

		public static final double kSpeed = 0.6;

		//Swerve最大速度。加速度
		public static final double kMaxVelocityMetersPerSecond = 5.0;
		public static final double kMaxAccelerationMetersPersecond = 3.0;
		public static final double kMaxAngularVelocityRadPerSecond = 4.3;

		//輪徑
		public static final double kwheeldiameterMeter = 0.1016;

		//動力齒輪比
		public static final double kThrottleGearRatio = 6.12;

		//動力速度轉換 通式：(1/齒輪比)*輪徑*PI   (這個也沒有搞懂要幹嘛)
		public static final double ThrottleVelocityConversionFactor =
			(Math.PI * kwheeldiameterMeter) / (60.0 * kThrottleGearRatio);

		public static final double ThrottlePositionConversionFactor =
			(Math.PI * kwheeldiameterMeter) / kThrottleGearRatio;

		public static final double kDefaultSpeed = 1;
		
		//電壓最大值
		public static final double kVoltagecompensation = 12.0;

		//最常超時時間
		public static final double kLongtimeoutMs = 100.0;

		public static final boolean kRotorMotorInverted = true;
		public static final boolean kThrottleMotorInverted = false;

		public static final double kThottleFF_kV = 2.35;
		public static final double kThottleFF_kA = 0.055; //0.48;
		
	
		public static final double kPath_kP = 0.008;//0.558;//0.537;
		public static final double kPath_kI = 0.6;//0.22;//0.12;//0.15;
		public static final double kPath_kD = 0;//0.00722;//0.1;//0.005;

		public static final double kPathZ_kP = 0.66;//1.297;//1.25;//1.29;
		public static final double kPathZ_kI = 0;//0;//0.05;//0.1;
		public static final double kPathZ_kD = 0;//0.001;//0.0025;

		public static final Translation2d[] kModuleoffsets = {
			new Translation2d(kwhatever/2, kwhatever/2),
			new Translation2d(kwhatever/2, kwhatever/-2),
			new Translation2d(kwhatever/-2, kwhatever/2),
			new Translation2d(kwhatever/-2, kwhatever/-2)
		};

		//質量KG
		public static final double kMass = 55;
		//轉動動量
		public static final double kMOI = 6.883;
		//輪子摩擦係數
		public static final double kWheelCOF = 1.2;
		//馬達電流限制
		public static final double kdriveCurrentLimit = 40;
		//每個輪子馬德數量
		public static final int kNumMotors = 1;

		public static final RobotConfig kconfig = new RobotConfig(
				kMass,
				kMOI,
				new ModuleConfig(
					kwheeldiameterMeter / 2,
					kMaxVelocityMetersPerSecond,
					kWheelCOF,
					DCMotor.getNEO(kNumMotors),
					kdriveCurrentLimit,
					kNumMotors
				),
				kModuleoffsets
			);
		}

	public static class OperatorConstants {
		public static final int kDriverControllerPort = 0;
		public static final int kActionControllerPort = 1;

		//Xbox控制器數值忽略基準值
		public static final double kControllerMinValue = 0.12;
	}
}
