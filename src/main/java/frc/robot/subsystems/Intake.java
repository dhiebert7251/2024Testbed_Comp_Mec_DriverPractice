// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {

  private final CANSparkMax m_intakeMotor = new CANSparkMax (IntakeConstants.kIntakeMotor,MotorType.kBrushed);
  private final CANSparkMax m_deployMotor = new CANSparkMax (IntakeConstants.kDeployMotor,MotorType.kBrushed);

  private final DigitalInput m_limitUp = new DigitalInput (IntakeConstants.kIntakeLimitUp);
  private final DigitalInput m_limitDown = new DigitalInput (IntakeConstants.kIntakeLimitUp);

  //declare sensor

  //Shuffleboard
  private final ShuffleboardTab intakeTab = Shuffleboard.getTab("Intake");
  private GenericEntry intakeLimitUp;
  private GenericEntry intakeLimitDown;

  /** Creates a new Intake. */
  public Intake() {

    m_intakeMotor.restoreFactoryDefaults();
    m_deployMotor.restoreFactoryDefaults();

    m_intakeMotor.setIdleMode(IdleMode.kBrake);
    m_deployMotor.setIdleMode(IdleMode.kBrake);

    m_intakeMotor.setInverted(IntakeConstants.kIntakeMotorReversed);
    m_deployMotor.setInverted(IntakeConstants.kDeployMotorReversed);

    //Shuffleboard
    intakeLimitUp = intakeTab.add("Intake Up",0).withWidget("Boolean Box").withSize(1,1).getEntry();
    intakeLimitDown = intakeTab.add("Intake Down",0).withWidget("Boolean Box").withSize(1,1).getEntry();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    showIntakeTelemetry();
  }



  public void showIntakeTelemetry(){
    intakeLimitUp.setBoolean(m_limitUp.get());
    intakeLimitDown.setBoolean(m_limitDown.get());
  }
}
