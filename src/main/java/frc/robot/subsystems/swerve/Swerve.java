
package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.SwerveConstants;
import frc.robot.utils.LimelightHelpers;

public class Swerve extends SubsystemBase{
    public final SwerveModule m_LeftFrontModule = new SwerveModule(
        SwerveConstants.kLeftFrontThrottleID,
        SwerveConstants.kLeftFrontRotorID,
        SwerveConstants.kLeftFrontEncoderID,
        SwerveConstants.kLeftFrontRotorEncoderOffset
    );

    public final SwerveModule m_LeftRearModule = new SwerveModule(
        SwerveConstants.kLeftRearThrottleID,
        SwerveConstants.kLeftRearRotorID,
        SwerveConstants.kLeftRearEncoderID,
        SwerveConstants.kLeftRearRotorEncoderOffset
    );

    public final SwerveModule m_RightFrontModule = new SwerveModule(
        SwerveConstants.kRightFrontThrottleID,
        SwerveConstants.kRightFrontRotorID,
        SwerveConstants.kRightFrontEncoderID,
        SwerveConstants.kRightFrontRotorEncoderOffset
    );
    

    public final SwerveModule  m_RightRearModule = new SwerveModule(
        SwerveConstants.kRightRearThrottleID,
        SwerveConstants.kRightRearRotorID,
        SwerveConstants.kRightRearEncoderID,
        SwerveConstants.kRightRearRotorEncoderOffset
    );
    
    private final Pigeon2 m_Pigeon = new Pigeon2(5, "SwerveCancoder");

    private Field2d m_field = new Field2d();

    private double maxSpeedRatio = SwerveConstants.kDefaultSpeed;
    private double headingOffset = 0;

    private Alliance m_alliance;

    private SwerveDrivePoseEstimator m_poseEstimator;
    private SwerveDriveOdometry m_odometry;

    private boolean fieldOriented = true;
    private boolean iscameraGotSomething = false;

    private Pose2d m_RobotPose;

    public Swerve() {
        // Allinace
        resetAllinace();
        System.out.println("My alliance" + m_alliance);

        m_Pigeon.reset();
    
        // Odometry & PoseEstimator
        m_odometry = new SwerveDriveOdometry(
            SwerveConstants.kSwerveDriveKinematics,
            getImuARotation2d(),//Rotation2d.fromDegrees(m_Pigeon.getYaw().getValueAsDouble())
            getModulePositions()
        );
        
        m_poseEstimator = new SwerveDrivePoseEstimator(
        SwerveConstants.kSwerveDriveKinematics,
        getImuARotation2d(),
        getModulePositions(),
        Pose2d.kZero,//initialPose,
        VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(99999999)),
        VecBuilder.fill(0.7, 0.7, Units.degreesToRadians(99999999))
        );

        // SmartDashboard
        SmartDashboard.putData(m_field);

        // #region AutoBuilder
        RobotConfig config;{
            try{
                config = RobotConfig.fromGUISettings();
            } catch (Exception e) {
                // Handle exception as needed
                config = SwerveConstants.kconfig;
                e.printStackTrace();
            }
        }

        AutoBuilder.configure(
            this::getPose,
            this::setPose,
            this::getSpeeds,
            this::driveChassis,
            new PPHolonomicDriveController(
                    new PIDConstants(
                            SwerveConstants.kPath_kP,
                            SwerveConstants.kPath_kI,
                            SwerveConstants.kPath_kD
                        ),
                    new PIDConstants(
                            SwerveConstants.kPathZ_kP,
                            SwerveConstants.kPathZ_kI,
                            SwerveConstants.kPathZ_kD
                        )
                ),config,
            () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                // if(DriverStation.getAlliance().isPresent()) {
                //     System.out.println("auto Alliance = RED =>" + (DriverStation.getAlliance().get() == DriverStation.Alliance.Red));}

                if (DriverStation.getAlliance().isPresent()) {
                    return DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
                }
                return false;
                //return true;
            },
            this
        );
        // #endregion
    }
    
    @Override
    public void periodic() {
        if(DriverStation.getAlliance().isPresent())
            m_alliance = DriverStation.getAlliance().get();

        m_poseEstimator.update(
            getImuARotation2d(),
            getModulePositions());

        m_odometry.update(
            getImuARotation2d(),
            getModulePositions());

        this.updateVisionPose();

        m_RobotPose = m_poseEstimator.getEstimatedPosition();

        m_field.setRobotPose(m_RobotPose);

        // #region SmartDashboard
        SmartDashboard.putNumber("LF", this.getModuleStates()[0].angle.getDegrees());
        SmartDashboard.putNumber("RF", this.getModuleStates()[1].angle.getDegrees());
        SmartDashboard.putNumber("LR", this.getModuleStates()[2].angle.getDegrees());
        SmartDashboard.putNumber("RR", this.getModuleStates()[3].angle.getDegrees());
        
        /*
        SmartDashboard.putNumber("speed", maxSpeedRatio);

        SmartDashboard.putNumber("LF_speed", getModuleStates()[0].speedMetersPerSecond);
        SmartDashboard.putNumber("LR_speed", getModuleStates()[2].speedMetersPerSecond);
        SmartDashboard.putNumber("RF_speed", getModuleStates()[1].speedMetersPerSecond);
        SmartDashboard.putNumber("RR_speed", getModuleStates()[3].speedMetersPerSecond);
        */

        /*SmartDashboard.putNumber("X Position", m_poseEstimator.getEstimatedPosition().getX());
        SmartDashboard.putNumber("Y Position", m_poseEstimator.getEstimatedPosition().getY());
        SmartDashboard.putNumber("Ange", getImuARotation2d().getDegrees());
        SmartDashboard.putNumber("getAnge", m_Pigeon.getYaw().getValueAsDouble());
        SmartDashboard.putNumber("EFTFRONTSPEED", m_LeftFrontModule.get());
        SmartDashboard.putNumber("EFTREARSPEED", m_LeftRearModule.get());
        SmartDashboard.putNumber("RFSPEED", m_RightFrontModule.get());
        SmartDashboard.putNumber("RRSPEED", m_RightRearModule.get());*/
        // #endregion
    }

    // #region IMU
    public void resetImu() {
        m_Pigeon.reset();
    }

    public Rotation2d getImuARotation2d() {
        if (m_alliance == Alliance.Blue) return m_Pigeon.getRotation2d().plus(Rotation2d.fromDegrees(180));
        return m_Pigeon.getRotation2d();
    }
    // #endregion

    // #region Allinace
    public void resetAllinace() {
        if(DriverStation.getAlliance().isPresent())
            m_alliance = DriverStation.getAlliance().get();
            System.out.println("Re Allinac" + m_alliance);
    }

    public Alliance getAlliance() {
        return m_alliance;
    }
    // #endregion

    // #region HeadingAngle
    public Command resetHeadingOffset() {
        return runOnce(() -> {
            this.headingOffset = m_Pigeon.getYaw().getValueAsDouble();
        });
    }

    public void setHeadingAngle(double heading) {
        this.headingOffset = heading;
    }
    // #endregion

    // #region PoseEstimator

    public void resetPoseEstimator(Rotation2d rotation, Pose2d pose) {
        m_poseEstimator.resetPosition(rotation, getModulePositions(), pose);
    }

    public void setPose(Pose2d pose) {
        m_odometry.resetPosition(
            getImuARotation2d(),
            getModulePositions(),
            pose);
        m_poseEstimator.resetPosition(
            getImuARotation2d(),
            getModulePositions(),
            pose);
    }

    public Pose2d getPose() {
        return m_poseEstimator.getEstimatedPosition();
    }
    // #endregion

    // #region m_field

    public void setAutoalignmentFieldOriented(Pose2d targetPsoe) {
        m_field.getObject("Autoalignment").setPose(targetPsoe);
    }
    //#endregion

    // #region ModuleState
    //將前面返回的state最大速度限制再回傳回去給SwerveModuleState
    public void setModulestate(SwerveModuleState[] desiredState) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredState, this.maxSpeedRatio * SwerveConstants.kMaxVelocityMetersPerSecond);
        
        m_LeftFrontModule.setState(desiredState[0]);
        m_RightFrontModule.setState(desiredState[1]);
        m_LeftRearModule.setState(desiredState[2]);
        m_RightRearModule.setState(desiredState[3]);
    }

    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[]{
            m_LeftFrontModule.getState(),
            m_RightFrontModule.getState(),
            m_LeftRearModule.getState(),
            m_RightRearModule.getState()
        };
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[]{
            m_LeftFrontModule.getPosition(),
            m_RightFrontModule.getPosition(),
            m_LeftRearModule.getPosition(),
            m_RightRearModule.getPosition()
        };
    }

    public ChassisSpeeds getSpeeds() {
        return SwerveConstants.kSwerveDriveKinematics.toChassisSpeeds(getModuleStates());
    }
    // #endregion

    // #region TargetPose
    // public Pose3d getTargetPose(int ID) {
    //     var tagPose = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
    //     if(tagPose.getTagPose(ID).isPresent()){
    //         return tagPose.getTagPose(ID).get();
    //     }
    //     return null;
    // }
    // #endregion

    // #region Vision
    public void updateVisionPose() {        
        LimelightHelpers.SetRobotOrientation("limelight", this.getImuARotation2d().getDegrees(), m_Pigeon.getAngularVelocityZWorld().getValueAsDouble(), 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        iscameraGotSomething = LimelightHelpers.getTV("");

        if(iscameraGotSomething && mt2 != null) {
            Pose2d visionPoseNoRot = new Pose2d(
                mt2.pose.getTranslation(),
                m_poseEstimator.getEstimatedPosition().getRotation()
            );

            m_poseEstimator.addVisionMeasurement(
                visionPoseNoRot,
                mt2.timestampSeconds); 
            // m_poseEstimator.resetPosition(getImuARotation2d(), getModulePositions(), mt2.pose);
        }
        SmartDashboard.putBoolean("cameraGotSomething", iscameraGotSomething);
    }
    // #endregion

    // #region drive
    /**
     * Drives the swerve - Input range: [-1, 1]
     * 
     * @param xSpeed percent power in the X direction (X 方向的功率百分比)
     * @param ySpeed percent power in the Y direction (Y 方向的功率百分比)
     * @param zSpeed percent power for rotation (旋轉的功率百分比)
     * @param fieldOriented configure robot movement style (設置機器運動方式) (field or robot oriented)
     */
    public void drive(double xSpeed, double ySpeed, double zSpeed, boolean fieldOriented) {
        SmartDashboard.putNumber("x_speed_set", xSpeed);
        SmartDashboard.putNumber("y_speed_set",  ySpeed);

        xSpeed *= SwerveConstants.kMaxVelocityMetersPerSecond;
        ySpeed *= SwerveConstants.kMaxVelocityMetersPerSecond;
        zSpeed *= SwerveConstants.kMaxAngularVelocityRadPerSecond;
        
        if (fieldOriented) {
            SwerveModuleState[] states = SwerveConstants.kSwerveDriveKinematics.toSwerveModuleStates(
                ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, zSpeed, Rotation2d.fromDegrees(m_Pigeon.getYaw().getValueAsDouble() - this.headingOffset)));
            setModulestate(states);
        } else {
            SwerveModuleState[] states = SwerveConstants.kSwerveDriveKinematics.toSwerveModuleStates(
                new ChassisSpeeds(xSpeed, ySpeed, zSpeed));
            setModulestate(states);
        }
    }

    public void drive(double xSpeed, double ySpeed, double zSpeed) {
        drive(xSpeed, ySpeed, zSpeed, fieldOriented);
    }

    public void driveChassis(ChassisSpeeds speeds) {
        if (speeds.vxMetersPerSecond != 0 || speeds.vyMetersPerSecond != 0) {
            driveChassis(
                -speeds.vxMetersPerSecond,
                -speeds.vyMetersPerSecond,
                -speeds.omegaRadiansPerSecond
            );
        } else {
            driveChassis(0,0,0);
        }
    }

    public void driveChassis(double xSpeed, double ySpeed, double zSpeed) { 
        drive(xSpeed, ySpeed, zSpeed, false);
    }

    public void autoDriver(double xSpeed, double ySpeed, double zSpeed) {
        drive(xSpeed, ySpeed, zSpeed, true);
    }
    // #endregion

    // #region Command
    public Command switchDriveMode() {
        return runOnce(() -> {
            fieldOriented = !fieldOriented;
        });
    }

    public Command increaseSpeed() {
        return runOnce(() -> {
            if(this.maxSpeedRatio > 0.9) return;
            this.maxSpeedRatio+=0.1;
        });
    }

    public Command decreaseSpeed() {
        return runOnce(() -> {
            if(this.maxSpeedRatio < 0.2) return;
            this.maxSpeedRatio-=0.1;
        });
    }

    public Command tohighSpeed() {
        return runOnce(() -> {
            this.maxSpeedRatio = 0.8;
        });
    }

    public Command tolowspeed() {
        return runOnce(() -> {
            this.maxSpeedRatio = 0.5;
        });
    }
    // #endregion
}