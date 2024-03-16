package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {

    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kDriveMotorGearRatio = 1 / 6.75;
        public static final double kTurningMotorGearRatio = 7 / 150.0;
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        public static final double kPTurning = 1.00;

        public static final int gyroPort = 11;

    }

    public static final class DriveConstants {

        public static final double kTrackWidth = Units.inchesToMeters(21.75);
        // Distance between right and left wheels
        public static final double kWheelBase = Units.inchesToMeters(21.75);
        // Distance between front and back wheels
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2));

                //SPARK MAXES
        public static final double driveBaseRadius = Units.inchesToMeters(21.75 * Math.sqrt(2) / 2);

        // drive 40 amps
        public static final int kFrontLeftDriveMotorPort = 7; //c
        public static final int kBackLeftDriveMotorPort = 9; //c
        public static final int kFrontRightDriveMotorPort = 1; //c
        public static final int kBackRightDriveMotorPort = 14; //c

        // turning 20 amps
        public static final int kFrontLeftTurningMotorPort = 3; //c
        public static final int kBackLeftTurningMotorPort = 8; //c
        public static final int kFrontRightTurningMotorPort = 2; //c
        public static final int kBackRightTurningMotorPort = 17; //c

        public static final boolean kFrontLeftTurningEncoderReversed = true;
        public static final boolean kBackLeftTurningEncoderReversed = true;
        public static final boolean kFrontRightTurningEncoderReversed = true;
        public static final boolean kBackRightTurningEncoderReversed = true;

        public static final boolean kFrontLeftDriveEncoderReversed = true;
        public static final boolean kBackLeftDriveEncoderReversed = true;
        public static final boolean kFrontRightDriveEncoderReversed = true;
        public static final boolean kBackRightDriveEncoderReversed = true;

        public static final int kFrontLeftDriveAbsoluteEncoderPort = 2;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 1;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 3;
        public static final int kBackRightDriveAbsoluteEncoderPort = 0;

        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 0.383;//3.545944; //2.76; //2.75762;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 4.294;//1.175327; //1.91; //1.97222;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 6.150;//3.021143;// 0.49; //0.488692;
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad =5.255; //2.095080; // 4.61; //4.64258;

        public static final double kPhysicalMaxSpeedMetersPerSecond = 4.60248;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = 4.5; //4.5; //kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = 2 * Math.PI;
               // kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 1.0;
        public static final double kMaxAngularSpeedRadiansPerSecond = //
                DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
        public static final double kPXController = 1.5;
        public static final double kPYController = 1.5;
        public static final double kPThetaController = 3;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(kMaxAngularSpeedRadiansPerSecond, kMaxAngularAccelerationRadiansPerSecondSquared);

         public static final HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(
            new PIDConstants(0.05, 0, 0), // Translation constants 
            new PIDConstants(0.025, 0, 0), // Rotation constants 
            1.0, //kMaxSpeedMetersPerSecond, 
            DriveConstants.driveBaseRadius, // Drive base radius (distance from center to furthest module) 
            new ReplanningConfig());
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;

        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 4;
        public static final int kDriverFieldOrientedButtonIdx = 1;

        public static final double kDeadband = 0.1;

        public static int XBX_L_X = 0;
        public static int XBX_L_Y = 1;
        public static int XBX_L_TRIG = 2;
        public static int XBX_R_TRIG = 3;
        public static int XBX_R_X = 4;
        public static int XBX_R_Y = 5;

        public static int XBX_A = 1;
        public static int XBX_B = 2;
        public static int XBX_C = 3;
        public static int XBX_D = 4;
        public static int LEFT_BUMPER = 5;
        public static int RIGHT_BUMPER = 6;

    }

    public static final class NoteHandlerConstants {
        public static final int LEFT_ARM_MOTOR_CONTROLLER_ID = 16;
        public static final int RIGHT_ARM_MOTOR_CONTROLLER_ID = 6;
        public static final int INTAKE_MOTOR_CONTROLLER_ID = 11;
        public static final int UPPER_SHOOTER_MOTOR_CONTROLLER_ID = 10;
        public static final int LOWER_SHOOTER_MOTOR_CONTROLLER_ID = 15;
        public static final int ARM_ENCODER = 0;

        public static final int SHOOTER_STATUS_FRAME_0_PERIOD = 10;
        public static final int SHOOTER_STATUS_FRAME_1_PERIOD = 20;
        public static final int SHOOTER_STATUS_FRAME_2_PERIOD = 56000;

        public static final int LOADER_STATUS_FRAME_0_PERIOD = 10;
        public static final int LOADER_STATUS_FRAME_1_PERIOD = 58000;
        public static final int LOADER_STATUS_FRAME_2_PERIOD = 57000;

        public static final boolean INVERT_INTAKE_ROLLER = false;
        public static final boolean INVERT_UPPER_SHOOTER = true;
        public static final boolean INVERT_LOWER_SHOOTER = true;
        public static final boolean INVERT_LEFT_ARM = false;
        public static final boolean INVERT_RIGHT_ARM = false;

        public static final double SHOOTER_KP = 0;
        public static final double SHOOTER_KI = 0;
        public static final double SHOOTER_KD = 0;
        public static final double SHOOTER_KFF = 0;
    
        public static final double SHOOTER_KS = 0.31063;
        public static final double SHOOTER_KV = 0.0021641;
        public static final double SHOOTER_KA = 0.00051441;

        public static final double a = 0.16479;
        public static final double b = 0.0021169;
        public static final double c = 0.00041652;

        public static final double SHOULDER_KS = 0;
        public static final double SHOULDER_KG = 0.88;
        public static final double SHOULDER_KV = 4.87;
        public static final double SHOULDER_KA = 0.07;

        public static final double ARM_KP = 0.4;
        public static final double ARM_KI = 0;
        public static final double ARM_KD = 0;
        public static final double ARM_KFF = 0;
    
        public static final double SHOOTER_SET_RPM = 4000;        
        public static final double SHOOTER_SPEED_TOLERANCE = 250;
        public static final double ANGLE_TOLERANCE = Math.toRadians(2);
        public static final double ARM_SET_ANGLE = Math.toRadians(5.2);
        public static final double ARM_START_ANGLE = 1.7;

        public static final double AMP_RPM = 500;
        public static final double AMP_ANGLE = 1.75;

        public static final double ARM_UPPER_LIMIT = 1.70;
        public static final double ARM_LOWER_LIMIT = 0;

        public static final double ARM_DEADBAND = Math.toRadians(0.5);

        public static final int SENSOR_ID = 0;

        public static final double ARM_RADIANS_PER_MOTOR_ROTATION = (1 / 250) * 2 * Math.PI;
    }

    public static final class VisionConstants {
        public static final InterpolatingDoubleTreeMap DISTANCE_TO_ANGLE_MAP = new InterpolatingDoubleTreeMap();

        static { DISTANCE_TO_ANGLE_MAP.put(30.0, .75); }
        public static final InterpolatingDoubleTreeMap DISTANCE_TO_SHOOT_MAP = new InterpolatingDoubleTreeMap();
        static { DISTANCE_TO_SHOOT_MAP.put(30.0, 4000.0); }
    }

    public static final class ClimberConstants {
        public static final int CLIMBER_1 = 5;
        public static final int CLIMBER_2 = 4;
        public static final int SWITCH = 10;
    }
}
