// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.NoteHandlerConstants;
import frc.robot.subsystems.NoteHandler;

public class OtherNoteShot extends Command {

  private final NoteHandler noteHandler;  
  private double intakeSpeed, shootingSpeed, encoderAngle;
  private double IN = 1.00;

  /** Creates a new BasicNoteShot. */
  public OtherNoteShot(NoteHandler noteHandler) {
      this.noteHandler = noteHandler;
      addRequirements(noteHandler);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

      intakeSpeed = 0;
      shootingSpeed = NoteHandlerConstants.SHOOTER_SET_RPM;
      encoderAngle = Math.toRadians(10);


        // once the shooter speed reaches the RPM and arm angle is set intake in again so the note zooms into the speaker

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    noteHandler.setIntakeSpeed(intakeSpeed);
    noteHandler.setShooterRPM(shootingSpeed);
    noteHandler.setArmAngle(encoderAngle);

    if ((Math.abs(shootingSpeed - noteHandler.getLowerShooterRPM()) <= NoteHandlerConstants.SHOOTER_SPEED_TOLERANCE)
        | (Math.abs(shootingSpeed - noteHandler.getUpperShooterRPM()) <= NoteHandlerConstants.SHOOTER_SPEED_TOLERANCE)
        & (Math.abs(encoderAngle - noteHandler.getArmAngle()) <= NoteHandlerConstants.ANGLE_TOLERANCE)) {
          intakeSpeed = IN;
      }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    noteHandler.setArmAngle(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return noteHandler.getLoaderSensor(); // FIGURE OUT HOW TO STOP THE SHOT
  }
}
