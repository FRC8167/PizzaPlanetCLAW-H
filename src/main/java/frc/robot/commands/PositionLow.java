// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Pivot;

public class PositionLow extends CommandBase {
  Arm arm;
  Pivot pivot;
  Grabber grabber;
  /** Creates a new PositionLow. */
  public PositionLow(Arm arm, Pivot pivot, Grabber grabber) {
    this.arm = arm;
    this.pivot = pivot;
    this.grabber = grabber;
    addRequirements(arm, pivot, grabber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    new SequentialCommandGroup(
      new SetPivotAngle(pivot, Constants.PIVOT_GROUND_13),
      new SetArmDistance(arm, Constants.ARM_GROUND_21),
      new InstantCommand(() -> grabber.openGrabber())
    );
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
