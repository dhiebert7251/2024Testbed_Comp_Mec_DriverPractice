// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class Climber extends SubsystemBase {
  private final CANSparkMax m_climbingMotor = new CANSparkMax (ClimberConstants.kClimbingMotor,MotorType.kBrushed);
  
  private final DigitalInput m_climberLimitUp = new DigitalInput (ClimberConstants.kClimberLimitUp);
  private final DigitalInput m_climberLimitDown = new DigitalInput (ClimberConstants.kClimberLimitDown);

  //Shuffleboard
  private final ShuffleboardTab climberTab = Shuffleboard.getTab("Climber");
  private GenericEntry climberLimitUp;
  private GenericEntry climberLimitDown;

  /** Creates a new Climber. */
  public Climber() {

    m_climbingMotor.restoreFactoryDefaults();

    m_climbingMotor.setIdleMode(IdleMode.kBrake);

    m_climbingMotor.setInverted(ClimberConstants.kClimbingMotorReversed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    showClimberTelemetry();
  }
}
