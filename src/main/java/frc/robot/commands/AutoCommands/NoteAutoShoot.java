package frc.robot.commands.AutoCommands;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.NoteHandlerConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.NoteHandler;

public class NoteAutoShoot extends Command {

  private final NoteHandler noteHandler; 
  private Timer timer;

  private double shootingSpeed;

  private double targetOffsetAngle_Vertical, angleToGoalDegrees, angleToGoalRadians, 
  distanceFromLimelightToGoalInches, encoderAngle,
  currentBottomShooterRPM, currentTopShooterRPM, currentArmAngle, intakeSpeed;

  // how many degrees back is your limelight rotated from perfectly vertical?
  double limelightMountAngleDegrees = 15.0; 
  // distance from the center of the Limelight lens to the floor
  double limelightLensHeightInches = 14.0; 
  // distance from the target to the floor
  double goalHeightInches = 52.0; 

  double IN = 0.50;
  double OUT = -0.50;

  /** Creates a new IntakeCargo. */
  public NoteAutoShoot(NoteHandler noteHandler) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.noteHandler = noteHandler;
    timer = new Timer();
    addRequirements(noteHandler);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeSpeed = 0.0;
    timer.reset();
    timer.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    targetOffsetAngle_Vertical = LimelightHelpers.getTY("limelight");
    angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
    angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

    distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians); //calculate distance

    shootingSpeed = noteHandler.getTargetRPM(distanceFromLimelightToGoalInches);
    encoderAngle = noteHandler.getTargetAngle(distanceFromLimelightToGoalInches);
    
    currentBottomShooterRPM = noteHandler.getLowerShooterRPM();
    currentTopShooterRPM = noteHandler.getUpperShooterRPM();
    currentArmAngle = noteHandler.getArmAngle();

    if ((Math.abs(shootingSpeed - currentBottomShooterRPM) <= NoteHandlerConstants.SHOOTER_SPEED_TOLERANCE)
        && (Math.abs(shootingSpeed - currentTopShooterRPM) <= NoteHandlerConstants.SHOOTER_SPEED_TOLERANCE)
        && (Math.abs(encoderAngle - currentArmAngle) <= NoteHandlerConstants.ANGLE_TOLERANCE)) {
          intakeSpeed = IN;
          timer.start();
    }

    noteHandler.setIntakeSpeed(intakeSpeed);
    noteHandler.setShooterRPM(shootingSpeed);
    noteHandler.setArmAngle(encoderAngle);
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (timer.get() > 2.00); //do a timer command to end this
  }
}
