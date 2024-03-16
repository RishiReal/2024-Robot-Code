package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class SwerveSubsystem extends SubsystemBase {
    //private final ADIS16470_IMU gyro;
    private final Pigeon2 gyro;
    private final SwerveDriveOdometry odometry;

   // private Field2d field = new Field2d();

    private SwerveModule frontLeft = new SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

       private SwerveModule frontRight = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
            DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);
        
      private SwerveModule  backLeft = new SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftTurningEncoderReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
            DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

       private SwerveModule backRight = new SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderPort,
            DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed);

    ChassisSpeeds chassisSpeeds;

    public SwerveSubsystem() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();

        gyro = new Pigeon2(0);
        //gyro.setInverted();
        odometry = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, getRotation2d(), getModulePositions());  

    AutoBuilder.configureHolonomic(
      this::getPose2d, 
      this::resetPose, 
      this::getSpeeds, 
      this::driveRobotRelative, 
      Constants.AutoConstants.pathFollowerConfig,
      () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
      },
      this
    );
}

    public void zeroHeading() {
        gyro.reset();
    }

    public SwerveModulePosition getFrontLeftModulePosition() {
        return new SwerveModulePosition(frontLeft.getDrivePosition(), getRotation2d());
    }

    public SwerveModulePosition getFrontRightModulePosition() {
        return new SwerveModulePosition(frontRight.getDrivePosition(), getRotation2d());
    }

    public SwerveModulePosition getBackLeftModulePosition() {
        return new SwerveModulePosition(backLeft.getDrivePosition(), getRotation2d());
    }

    public SwerveModulePosition getBackRightModulePosition() {
        return new SwerveModulePosition(backRight.getDrivePosition(), getRotation2d());
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
        getFrontLeftModulePosition(),
        getFrontRightModulePosition(),
        getBackLeftModulePosition(),
        getBackRightModulePosition()
      };
    }

    public void resetPose(Pose2d pose) {
        odometry.resetPosition(getRotation2d(), getModulePositions(), pose);
  }
  
  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

    SwerveModuleState[] targetStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(targetSpeeds);
    setModuleStates(targetStates);
  }

    public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
        driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getPose2d().getRotation()));
    }

    public ChassisSpeeds getSpeeds() {
        return DriveConstants.kDriveKinematics.toChassisSpeeds(frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState());
    }
    
    public double getHeading() {
    //return Math.IEEEremainder(gyro.getAngle(IMUAxis.kYaw), 360);
       return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    public double getTurnRate() {
        return gyro.getRate();
    }
    
    public Rotation2d getRotation2d() {
        return gyro.getRotation2d();
    }
 
    public Pose2d getPose2d() {
        return odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(getRotation2d(), getModulePositions(), getPose2d());
    }

    @Override
    public void periodic() {
      odometry.update(getRotation2d(), getModulePositions());
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putNumber("Absolute Encoder Left Front", frontLeft.getAbsoluteEncoderRad()); //front left
        SmartDashboard.putNumber("Absolute Encoder Right Front", frontRight.getAbsoluteEncoderRad()); //front right
        SmartDashboard.putNumber("Absolute Encoder Left Back", backLeft.getAbsoluteEncoderRad()); //back left
        SmartDashboard.putNumber("Absolute Encoder Right Back", backRight.getAbsoluteEncoderRad()); //back right
        SmartDashboard.putString("Robot Location", getPose2d().getTranslation().toString());
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[1]); //1
        frontRight.setDesiredState(desiredStates[0]); //041
        backLeft.setDesiredState(desiredStates[3]); //3
        backRight.setDesiredState(desiredStates[2]); //2
    }
}
