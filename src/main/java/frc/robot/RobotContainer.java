// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;

import frc.robot.subsystems.swerve.Swerve;
import frc.robot.utils.OpzXboxController;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotContainer {
	private final Swerve m_Swerve = new Swerve();
 
	private final OpzXboxController m_DriveController = new OpzXboxController(
		OperatorConstants.kDriverControllerPort,
		OperatorConstants.kControllerMinValue);
	// private final OpzXboxController m_ActionController = new OpzXboxController(
	// 	OperatorConstants.kActionControllerPort,
	// 	OperatorConstants.kControllerMinValue);

	private final SendableChooser<Command> m_autoChooser;

	private AllianceStationID m_location;
	private Pose2d initialPose;

	public RobotContainer() {
		DataLogManager.start();
		DataLog log = DataLogManager.getLog();
		DriverStation.startDataLog(log);

		m_autoChooser = AutoBuilder.buildAutoChooser();
		SmartDashboard.putData(m_autoChooser);
		SmartDashboard.putData(CommandScheduler.getInstance());

		m_Swerve.setDefaultCommand(Commands.run(
			() -> m_Swerve.drive(
				m_DriveController.getLeftY(),
				m_DriveController.getLeftX(),
				m_DriveController.getRightX()
				),
				m_Swerve).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
		
		m_DriveController.x().or(m_DriveController.b()).whileFalse(Commands.runOnce(
			() -> m_Swerve.setDefaultCommand(Commands.run(
				() -> m_Swerve.drive(
					m_DriveController.getLeftY(),
					m_DriveController.getLeftX(),
					m_DriveController.getRightX()
				),
				m_Swerve))
		));
		
		configureBindings();
	}

	private void configureBindings() {
				
		m_DriveController.leftTrigger().onTrue(m_Swerve.tolowspeed());
		m_DriveController.rightTrigger().onTrue(m_Swerve.tohighSpeed());

		m_DriveController.rightBumper().onTrue(m_Swerve.increaseSpeed());
		m_DriveController.leftBumper().onTrue(m_Swerve.decreaseSpeed());

		m_DriveController.start().onTrue(m_Swerve.resetHeadingOffset());

		m_DriveController.povDown().onTrue(Commands.runOnce(
			() -> CommandScheduler.getInstance().cancelAll()
		));
	}

	public void robotInit() {
		m_location = DriverStation.getRawAllianceStation();

		switch (m_location) {
			case Blue1:
					initialPose = new Pose2d(7.506, 5.370, Rotation2d.fromDegrees(180));
					break;
			case Blue2:
					initialPose = new Pose2d(7.506, 4.0259, Rotation2d.fromDegrees(180));
					break;
			case Blue3:
					initialPose = new Pose2d(7.506, 1.910, Rotation2d.fromDegrees(180));
					break;
			case Red3:
					initialPose = new Pose2d(10.206944, 5.370, Rotation2d.fromDegrees(0));
					break;
			case Red2:
					initialPose = new Pose2d(10.206944, 4.0259, Rotation2d.fromDegrees(0));
					break;
			case Red1:
					initialPose = new Pose2d(10.206944, 1.910, Rotation2d.fromDegrees(0));
					break;
			default:
					initialPose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
					break;
			}

			m_Swerve.resetAllinace();
			m_Swerve.resetPoseEstimator(m_Swerve.getImuARotation2d(), initialPose);
	}

	public void enable() {
		m_Swerve.resetAllinace();
	}

	public void disable() {}

	public Command getAutonomousCommand() {
		//return new autoCommand(m_Swerve);
		return m_autoChooser.getSelected();
	}
}
