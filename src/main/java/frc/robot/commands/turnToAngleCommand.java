// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class turnToAngleCommand extends Command {
  /** Creates a new turnToAngleCommand. */
    private final Drivetrain drivetrain;
    private final double targetAngle;

  public turnToAngleCommand(Drivetrain drivetrain, double targetAngle) {
        this.drivetrain = drivetrain;
        this.targetAngle = targetAngle;
        addRequirements(drivetrain);
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.enableTurnToAngleMode();
    drivetrain.turnToAngle(targetAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.disableTurnToAngleMode();
    }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return drivetrain.atTargetAngle();

  }
}
