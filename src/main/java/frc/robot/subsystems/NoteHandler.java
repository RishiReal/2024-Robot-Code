// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Heavily Inspired by FRC Team 95 Code Skeleton - thank you!

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.NoteHandlerConstants;
import frc.robot.Constants.VisionConstants;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

public class NoteHandler extends SubsystemBase {

  private final CANSparkMax lowerShooter,leftArm,rightArm, upperShooter, intake;
  private SysIdRoutine routine;

  private final SparkPIDController upperShooterPID, lowerShooterPID, leftArmPID, rightArmPID;
  private final SimpleMotorFeedforward flywheelFeedforward; 
  private final SimpleMotorFeedforward flywheelFeedforward2;
  private final ArmFeedforward armFeedforward;
  private final RelativeEncoder upperShooterEncoder, lowerShooterEncoder, leftMotorArmEncoder, rightMotorArmEncoder, intakeEncoder;
  private final DutyCycleEncoder armEncoder;
  private final DigitalInput intakeSensor;
  private final TrapezoidProfile shoulderProfile;

  private final double armAngleStart;

  private double armGoal, armFeedforwardValue;
  
  private final Timer timer;
  private TrapezoidProfile.State armSetpoint, profileStart;

  public NoteHandler() {
    intake = new CANSparkMax(NoteHandlerConstants.INTAKE_MOTOR_CONTROLLER_ID, MotorType.kBrushless);
    lowerShooter = new CANSparkMax(NoteHandlerConstants.LOWER_SHOOTER_MOTOR_CONTROLLER_ID, MotorType.kBrushless);
    leftArm = new CANSparkMax(NoteHandlerConstants.LEFT_ARM_MOTOR_CONTROLLER_ID, MotorType.kBrushless);
    rightArm = new CANSparkMax(NoteHandlerConstants.RIGHT_ARM_MOTOR_CONTROLLER_ID, MotorType.kBrushless);
    upperShooter = new CANSparkMax(NoteHandlerConstants.UPPER_SHOOTER_MOTOR_CONTROLLER_ID, MotorType.kBrushless);

    intakeSensor = new DigitalInput(NoteHandlerConstants.SENSOR_ID);

    upperShooter.restoreFactoryDefaults();
    lowerShooter.restoreFactoryDefaults();
    leftArm.restoreFactoryDefaults();
    rightArm.restoreFactoryDefaults();
    intake.restoreFactoryDefaults();

    upperShooterPID = upperShooter.getPIDController(); 
    lowerShooterPID = lowerShooter.getPIDController();
    leftArmPID = leftArm.getPIDController();
    rightArmPID = rightArm.getPIDController();

    lowerShooter.setIdleMode(IdleMode.kCoast);
    upperShooter.setIdleMode(IdleMode.kCoast);
    intake.setIdleMode(IdleMode.kBrake);
    leftArm.setIdleMode(IdleMode.kBrake);
    rightArm.setIdleMode(IdleMode.kBrake);
  
    lowerShooter.setSmartCurrentLimit(40);
    upperShooter.setSmartCurrentLimit(40);

    intake.setSmartCurrentLimit(20);

    leftArm.setSmartCurrentLimit(40);
    rightArm.setSmartCurrentLimit(40);

    lowerShooter.setOpenLoopRampRate(0.5);
    upperShooter.setOpenLoopRampRate(0.5);
    upperShooter.setClosedLoopRampRate(0.5);
    lowerShooter.setClosedLoopRampRate(0.5);

    rightArm.setOpenLoopRampRate(0.5);
    rightArm.setClosedLoopRampRate(0.5);
    leftArm.setOpenLoopRampRate(0.5);
    rightArm.setClosedLoopRampRate(0.5);

    leftArm.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 5);

    lowerShooter.setPeriodicFramePeriod(PeriodicFrame.kStatus0, NoteHandlerConstants.SHOOTER_STATUS_FRAME_0_PERIOD);
    lowerShooter.setPeriodicFramePeriod(PeriodicFrame.kStatus1, NoteHandlerConstants.SHOOTER_STATUS_FRAME_1_PERIOD);
    lowerShooter.setPeriodicFramePeriod(PeriodicFrame.kStatus2, NoteHandlerConstants.SHOOTER_STATUS_FRAME_2_PERIOD);

    upperShooter.setPeriodicFramePeriod(PeriodicFrame.kStatus0, NoteHandlerConstants.SHOOTER_STATUS_FRAME_0_PERIOD);
    upperShooter.setPeriodicFramePeriod(PeriodicFrame.kStatus1, NoteHandlerConstants.SHOOTER_STATUS_FRAME_1_PERIOD);
    upperShooter.setPeriodicFramePeriod(PeriodicFrame.kStatus2, NoteHandlerConstants.SHOOTER_STATUS_FRAME_2_PERIOD);

    intake.setPeriodicFramePeriod(PeriodicFrame.kStatus0, NoteHandlerConstants.LOADER_STATUS_FRAME_0_PERIOD);
    intake.setPeriodicFramePeriod(PeriodicFrame.kStatus1, NoteHandlerConstants.LOADER_STATUS_FRAME_1_PERIOD);
    intake.setPeriodicFramePeriod(PeriodicFrame.kStatus2, NoteHandlerConstants.LOADER_STATUS_FRAME_2_PERIOD);
    
    intake.setInverted(NoteHandlerConstants.INVERT_INTAKE_ROLLER);
    upperShooter.setInverted(NoteHandlerConstants.INVERT_UPPER_SHOOTER);
    lowerShooter.setInverted(NoteHandlerConstants.INVERT_LOWER_SHOOTER);
    
    rightArm.follow(leftArm, true);
    //leftArm.setInverted(false);
    //rightArm.setInverted(true);

    upperShooterEncoder = upperShooter.getEncoder();
    lowerShooterEncoder = lowerShooter.getEncoder();
    leftMotorArmEncoder = leftArm.getEncoder();
    rightMotorArmEncoder = rightArm.getEncoder();
    intakeEncoder = intake.getEncoder();

    armEncoder = new DutyCycleEncoder(4);

    upperShooterPID.setP(NoteHandlerConstants.SHOOTER_KP);
    upperShooterPID.setI(NoteHandlerConstants.SHOOTER_KI);
    upperShooterPID.setD(NoteHandlerConstants.SHOOTER_KD);
    upperShooterPID.setFF(NoteHandlerConstants.SHOOTER_KFF);

    lowerShooterPID.setP(NoteHandlerConstants.SHOOTER_KP);
    lowerShooterPID.setI(NoteHandlerConstants.SHOOTER_KI);
    lowerShooterPID.setD(NoteHandlerConstants.SHOOTER_KD);
    lowerShooterPID.setFF(NoteHandlerConstants.SHOOTER_KFF);

    leftArmPID.setP(NoteHandlerConstants.ARM_KP);
    leftArmPID.setI(NoteHandlerConstants.ARM_KI);
    leftArmPID.setD(NoteHandlerConstants.ARM_KD);
    leftArmPID.setFF(NoteHandlerConstants.ARM_KFF);

    rightArmPID.setP(NoteHandlerConstants.ARM_KP);
    rightArmPID.setI(NoteHandlerConstants.ARM_KI);
    rightArmPID.setD(NoteHandlerConstants.ARM_KD);
    rightArmPID.setFF(NoteHandlerConstants.ARM_KFF);

    flywheelFeedforward = new SimpleMotorFeedforward(NoteHandlerConstants.SHOOTER_KS, NoteHandlerConstants.SHOOTER_KV, NoteHandlerConstants.SHOOTER_KA);
    flywheelFeedforward2 = new SimpleMotorFeedforward(NoteHandlerConstants.a, NoteHandlerConstants.b, NoteHandlerConstants.c);

    armFeedforward = new ArmFeedforward(
      NoteHandlerConstants.SHOULDER_KS,
      NoteHandlerConstants.SHOULDER_KG,
      NoteHandlerConstants.SHOULDER_KV,
      NoteHandlerConstants.SHOULDER_KA);

    leftArmPID.setOutputRange(-0.2, 1.0);
    rightArmPID.setOutputRange(-0.2, 1.0);

    armAngleStart = getArmAngle();

    shoulderProfile = new TrapezoidProfile(
        new TrapezoidProfile.Constraints(
            2,
            3));
    armGoal = armAngleStart;
    profileStart = new TrapezoidProfile.State(armGoal, 0);

    armFeedforwardValue = 0;

    lowerShooter.burnFlash();
    upperShooter.burnFlash();
    leftArm.burnFlash();
    rightArm.burnFlash();
    intake.burnFlash();

    routine = new SysIdRoutine(
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism(
            (Measure<Voltage> volts) -> {
              upperShooter.setVoltage(volts.in(Volts));
            },
            log -> {
              log.motor("upperShooter")
                  .voltage(Volts.of(upperShooter.getAppliedOutput() * upperShooter.getBusVoltage()))
                  .angularPosition(Rotations.of(getArmAngle()))
                  .angularVelocity(RotationsPerSecond.of( upperShooterEncoder.getVelocity()));
            },
            this));

  //encoderInitial = (-armEncoder.getAbsolutePosition() * 2 * Math.PI) + 4.686209;

  leftMotorArmEncoder.setPositionConversionFactor(0.00369444444 * 2 * Math.PI);
  leftMotorArmEncoder.setVelocityConversionFactor(0.00369444444 * 2 * Math.PI / 60);
  rightMotorArmEncoder.setPositionConversionFactor(0.00369444444 * 2 * Math.PI);
  rightMotorArmEncoder.setVelocityConversionFactor(0.00369444444 * 2 * Math.PI / 60);

  leftMotorArmEncoder.setPosition(1);
  rightMotorArmEncoder.setPosition(1);

  timer = new Timer();
  timer.start();

  }

  

  @Override
  public void periodic() {

  leftMotorArmEncoder.setPosition(getArmAngle());
  rightMotorArmEncoder.setPosition(getArmAngle());

    armSetpoint = shoulderProfile.calculate(
        timer.get(),
        profileStart,
        new TrapezoidProfile.State(armGoal, 0));

    armSetpoint = (armSetpoint.position >= NoteHandlerConstants.ARM_UPPER_LIMIT)
        ? new TrapezoidProfile.State(NoteHandlerConstants.ARM_UPPER_LIMIT, 0)
        : armSetpoint;

    if (Math.abs(armSetpoint.position - NoteHandlerConstants.ARM_LOWER_LIMIT) <= NoteHandlerConstants.ARM_DEADBAND) {
      leftArm.set(0);
      armFeedforwardValue = 0;
    } else {
      armFeedforwardValue = armFeedforward.calculate(armSetpoint.position, armSetpoint.velocity);
      leftArmPID.setReference(
          armSetpoint.position,
          ControlType.kPosition,
          0,
          armFeedforwardValue,
          ArbFFUnits.kVoltage);
    }

    // This method will be called once per scheduler run
    SmartDashboard.putNumber("UpperShooterVolts", upperShooter.get() * upperShooter.getBusVoltage());
    SmartDashboard.putNumber("LowerShooterVolts", lowerShooter.get() * lowerShooter.getBusVoltage());

    SmartDashboard.putNumber("LeftArmVolts", leftArm.get() * leftArm.getBusVoltage());
    SmartDashboard.putNumber("RightArmVolts", rightArm.get() * rightArm.getBusVoltage());

    SmartDashboard.putNumber("RealLowerShooterRPM", getLowerShooterRPM());
    SmartDashboard.putNumber("RealUpperShooterRPM", getUpperShooterRPM());

    SmartDashboard.putNumber("GetArmAngle", getArmAngle());
    SmartDashboard.putNumber("Right Arm Encoder Value", rightMotorArmEncoder.getPosition());
    SmartDashboard.putNumber("Left Arm Encoder Value", leftMotorArmEncoder.getPosition());

    SmartDashboard.putBoolean("Intake Sensor", getLoaderSensor());

    SmartDashboard.putNumber("IntakeRPM", getIntakeRPM());
  }

  public void setIntakeSpeed(double speed){
    intake.set(speed);
  }
  public void setShooterRPM(double RPM){
    lowerShooterPID.setReference(RPM, ControlType.kVelocity , 0, flywheelFeedforward2.calculate(RPM), ArbFFUnits.kVoltage);
    upperShooterPID.setReference(RPM, ControlType.kVelocity , 0, flywheelFeedforward.calculate(RPM), ArbFFUnits.kVoltage);
  }
  public void setShooterSpeed(double speed) {
    lowerShooter.set(speed);
    upperShooter.set(speed);
  }
  public double getLowerShooterRPM(){
    return lowerShooterEncoder.getVelocity();
  }
  public double getUpperShooterRPM(){
    return upperShooterEncoder.getVelocity();
  }
  public double getIntakeRPM() {
    return intakeEncoder.getVelocity();
  }
  public double getEncoderPosition(DutyCycleEncoder encoder){
    return (-encoder.getAbsolutePosition() * 2 * Math.PI) + 4.686209; // put code here to convert it to normal values
  }
  public double getArmAngle() {
    return getEncoderPosition(armEncoder);
  }
  public void setArmVoltage(double volts) {
    leftArm.setVoltage(volts);
  }
  public void setArmSpeed(double speed) {
    leftArm.set(speed);
  }
  public void setArmAngle(double angle) {
    if (angle != armGoal) {
      armGoal = angle;
      profileStart = new TrapezoidProfile.State(getArmAngle(), ((leftMotorArmEncoder.getVelocity() + rightMotorArmEncoder.getVelocity()) / 2));
      timer.stop();
      timer.reset();
      timer.start();
    }
  }
  public double getTargetRPM(double distance) {
    return VisionConstants.DISTANCE_TO_SHOOT_MAP.get(distance); //make an equation later or a table
  }
  public double getTargetAngle(double distance) {
    return VisionConstants.DISTANCE_TO_ANGLE_MAP.get(distance); //make equation later or a table
  }
  public boolean getLoaderSensor(){
    return !intakeSensor.get();
  }
  public void setPositionEncoders() {
    leftMotorArmEncoder.setPosition(getEncoderPosition(armEncoder));
    rightMotorArmEncoder.setPosition(getEncoderPosition(armEncoder));
  }
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return routine.quasistatic(direction);
  }
  
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return routine.dynamic(direction);
  }

}
