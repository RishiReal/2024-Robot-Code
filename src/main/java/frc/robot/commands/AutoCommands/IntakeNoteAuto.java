// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NoteHandler;

public class IntakeNoteAuto extends Command {

  private final NoteHandler noteHandler;
  private double IN = 0.50;
  private double intakeSpeed, encoderAngle;

  /** Creates a new IntakeNoteAuto. */
  public IntakeNoteAuto(NoteHandler noteHandler) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.noteHandler = noteHandler;
    addRequirements(noteHandler);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeSpeed = IN; 
    encoderAngle = Math.toRadians(0.0);

    noteHandler.setArmAngle(encoderAngle);
    noteHandler.setIntakeSpeed(intakeSpeed);
    noteHandler.setShooterRPM(0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    noteHandler.setArmAngle(Math.toRadians(10));
    noteHandler.setIntakeSpeed(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
