// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.NoteHandlerConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.NoteHandler;
import frc.robot.LimelightHelpers;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;


/** An example command that uses an example subsystem. */
public class NoteHandlerCommand extends Command {
  private final NoteHandler noteHandler;
  private final XboxController gamepad;

  private enum State {
    SHOOTING_SET, SHOOTING_VISION, IDLE, INTAKE_IN, INTAKE_OUT, HOLDING, AMP, SPOOL, RESET_INTAKE, ESCAPE
  }

  private State currentState;
  private double intakeSpeed, shootingSpeed;
  private boolean intakeIN, intakeOUT, shooterButtonVision, shooterButtonSet, ampButton, sensorValue, spoolButton, intakeStay, ampShotButton, escapeButton, manualArmButton, isNoteIn;

  private double targetOffsetAngle_Vertical, angleToGoalDegrees, angleToGoalRadians, 
  distanceFromLimelightToGoalInches, targetRPM, targetEncoderAngle, encoderAngle,
  currentBottomShooterRPM, currentTopShooterRPM, currentArmAngle;

  // how many degrees back is your limelight rotated from perfectly vertical?
  double limelightMountAngleDegrees = 15.0; 
  // distance from the center of the Limelight lens to the floor
  double limelightLensHeightInches = 14.0; 
  // distance from the 6target to the floor
  double goalHeightInches = 52.0; 

  double IN =.50;
  double OUT = -.40;
  
  public NoteHandlerCommand(NoteHandler noteHandler, XboxController gamepad) {
    this.noteHandler = noteHandler;
    this.gamepad = gamepad;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(noteHandler);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentState = State.IDLE;

    isNoteIn = false;

    SmartDashboard.putBoolean("NoteInside", isNoteIn);

    noteHandler.setShooterSpeed(0);
    noteHandler.setArmAngle(0);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    targetOffsetAngle_Vertical = LimelightHelpers.getTY("limelight");
    angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
    angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

    distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians); //calculate distance

    sensorValue = noteHandler.getLoaderSensor();

    targetRPM = noteHandler.getTargetRPM(distanceFromLimelightToGoalInches);
    targetEncoderAngle = noteHandler.getTargetAngle(distanceFromLimelightToGoalInches);

    currentBottomShooterRPM = noteHandler.getLowerShooterRPM();
    currentTopShooterRPM = noteHandler.getUpperShooterRPM();
    currentArmAngle = noteHandler.getArmAngle();

    shooterButtonVision = gamepad.getRawButton(OIConstants.XBX_A);
    shooterButtonSet = (gamepad.getRawButton(OIConstants.XBX_C));
    intakeIN = (gamepad.getRawAxis(OIConstants.XBX_R_TRIG) > 0.05);
    intakeOUT = (gamepad.getRawAxis(OIConstants.XBX_L_TRIG) > 0.05);
    ampButton = (gamepad.getRawButton(OIConstants.XBX_D));
    spoolButton = (gamepad.getRawButton(OIConstants.RIGHT_BUMPER));
    intakeStay = (gamepad.getRawButton(OIConstants.LEFT_BUMPER));
    ampShotButton = (gamepad.getRawButton(OIConstants.XBX_B));
    escapeButton = (gamepad.getPOV() == 90);
    manualArmButton = false;
    
    if (sensorValue) {
      isNoteIn = true;
    }
    
    SmartDashboard.putBoolean("NoteInside", isNoteIn);

    intakeSpeed = 0;
    shootingSpeed = 0;
    encoderAngle = 0;

    switch(currentState){      

      
      case ESCAPE:
        intakeSpeed = 0;
        encoderAngle = 1.70;

        if(!escapeButton) {
          currentState = State.HOLDING;
        }

      break;

      case INTAKE_IN:
        intakeSpeed = gamepad.getRawAxis(OIConstants.XBX_R_TRIG / 2);
        encoderAngle = Math.toRadians(0);

        if (!intakeIN) {
          currentState = State.HOLDING;
        }

        if (isNoteIn) {
          intakeSpeed = 0;
        }

      break;

      case RESET_INTAKE:
        isNoteIn = false;

        if (!intakeStay) {
          currentState = State.HOLDING;
        }

      case INTAKE_OUT:
        intakeSpeed = -gamepad.getRawAxis(OIConstants.XBX_L_TRIG / 2);
        encoderAngle = 0;

        if (!intakeOUT) {
          currentState = State.HOLDING;
        }

      break;

      case SHOOTING_VISION:
        intakeSpeed = 0;

        if (LimelightHelpers.getTV("limelight")) {
        encoderAngle = targetEncoderAngle;
        shootingSpeed = targetRPM;
        } else {
        encoderAngle = NoteHandlerConstants.ARM_SET_ANGLE;
        shootingSpeed = 4000;
        }
        // once the shooter speed reaches the RPM and arm angle is set intake in again so the note zooms into the speaker

      if ((Math.abs(shootingSpeed - currentBottomShooterRPM) <= NoteHandlerConstants.SHOOTER_SPEED_TOLERANCE)
        && (Math.abs(shootingSpeed - currentTopShooterRPM) <= NoteHandlerConstants.SHOOTER_SPEED_TOLERANCE)
        && (Math.abs(encoderAngle - currentArmAngle) <= NoteHandlerConstants.ANGLE_TOLERANCE)) {
          isNoteIn = false;
          intakeSpeed = IN;
        }  

        if (!shooterButtonVision) {
          currentState = State.HOLDING;
        }
      break;

      case SHOOTING_SET:
        intakeSpeed = 0;
        encoderAngle = NoteHandlerConstants.ARM_SET_ANGLE;
        shootingSpeed = 4000;

        // once the shooter speed reaches the RPM and arm angle is set intake in again so the note zooms into the speaker
        if ((Math.abs(shootingSpeed - currentBottomShooterRPM) <= NoteHandlerConstants.SHOOTER_SPEED_TOLERANCE)
        && (Math.abs(shootingSpeed - currentTopShooterRPM) <= NoteHandlerConstants.SHOOTER_SPEED_TOLERANCE) 
        && (Math.abs(encoderAngle - currentArmAngle) <= NoteHandlerConstants.ANGLE_TOLERANCE)) {
          isNoteIn = false;
          intakeSpeed = IN;
        }
        
        if (!shooterButtonSet) {
          currentState = State.HOLDING;
        }

      break;

      case SPOOL:
        intakeSpeed = 0;
        
        if (LimelightHelpers.getTV("limelight")) {
        encoderAngle = targetEncoderAngle;
        shootingSpeed = targetRPM;
        } else {
        encoderAngle = NoteHandlerConstants.ARM_SET_ANGLE;
        shootingSpeed = 4000;
        }

        if (!spoolButton) {
          currentState = State.HOLDING;
        }
      break;

      case AMP:
        intakeSpeed = 0;
        encoderAngle = NoteHandlerConstants.AMP_ANGLE;

        // once the shooter speed reaches the RPM and arm angle is set intake in again so the note zooms into the amp
        if (!ampButton) {
          currentState = State.HOLDING;
        }
        if (ampButton && ampShotButton) {
                shootingSpeed = NoteHandlerConstants.AMP_RPM;
                intakeSpeed = IN;
                isNoteIn = false;
          }
      
      break;

      case HOLDING:
        intakeSpeed = 0;
        shootingSpeed = 0;
        encoderAngle = Math.toRadians(10); //dont want it on the ground 
        if (intakeIN) {
          currentState = State.INTAKE_IN;
        }
        if (intakeOUT) {
          currentState = State.INTAKE_OUT;
        }
        if (shooterButtonVision) {
          currentState = State.SHOOTING_VISION;
        }
        if (shooterButtonSet) {
          currentState = State.SHOOTING_SET;
        }
        if (ampButton) {
          currentState = State.AMP;
        }
        if(spoolButton) {
          currentState = State.SPOOL;
        }
        if(intakeStay) {
          currentState = State.RESET_INTAKE;
        }
        if(escapeButton) {
          currentState = State.ESCAPE;
        }
      
        break;

      default:
        intakeSpeed = 0;
        shootingSpeed = 0;
        encoderAngle = Math.toRadians(10);

        if (intakeIN) {
          currentState = State.INTAKE_IN;
        }
        if (intakeOUT) {
          currentState = State.INTAKE_OUT;
        }
        if (shooterButtonVision) {
          currentState = State.SHOOTING_VISION;
        }
        if (shooterButtonSet) {
          currentState = State.SHOOTING_SET;
        }
        if (ampButton) {
          currentState = State.AMP;
        }
        if(spoolButton) {
          currentState = State.SPOOL;
        }
        if(intakeStay) {
          currentState = State.RESET_INTAKE;
        }
        if(escapeButton) {
          currentState = State.ESCAPE;
        }
      }
    
    noteHandler.setIntakeSpeed(intakeSpeed);
    noteHandler.setShooterRPM(shootingSpeed);
    noteHandler.setArmAngle(encoderAngle);

    SmartDashboard.putString("currentState", currentState.toString());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
