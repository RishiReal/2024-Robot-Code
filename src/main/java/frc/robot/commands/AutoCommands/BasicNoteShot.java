// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.NoteHandlerConstants;
import frc.robot.subsystems.NoteHandler;

public class BasicNoteShot extends Command {

  private final NoteHandler noteHandler;  
  private double intakeSpeed, shootingSpeed, encoderAngle;
  private double IN = .50;
  private Timer timer;

  /** Creates a new BasicNoteShot. */
  public BasicNoteShot(NoteHandler noteHandler) {
      this.noteHandler = noteHandler;
      addRequirements(noteHandler);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

      intakeSpeed = 0;
      shootingSpeed = NoteHandlerConstants.SHOOTER_SET_RPM;
      encoderAngle = NoteHandlerConstants.ARM_SET_ANGLE;
        // once the shooter speed reaches the RPM and arm angle is set intake in again so the note zooms into the speaker
      timer.reset();
      timer.stop();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    noteHandler.setIntakeSpeed(intakeSpeed);
    noteHandler.setShooterRPM(shootingSpeed);
    noteHandler.setArmAngle(encoderAngle);

    if ((Math.abs(shootingSpeed - noteHandler.getLowerShooterRPM()) <= NoteHandlerConstants.SHOOTER_SPEED_TOLERANCE)
        && (Math.abs(shootingSpeed - noteHandler.getUpperShooterRPM()) <= NoteHandlerConstants.SHOOTER_SPEED_TOLERANCE)
        && (Math.abs(encoderAngle - noteHandler.getArmAngle()) <= NoteHandlerConstants.ANGLE_TOLERANCE)) {
          intakeSpeed = IN;
      }

    if (intakeSpeed == IN) {
      timer.start();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    noteHandler.setArmAngle(10);
    noteHandler.setIntakeSpeed(0);
    noteHandler.setShooterRPM(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (timer.get() > 2.00);
  }
}
