// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Shooter extends SubsystemBase {

  private final CANSparkMax m_leftFeedMotor = new CANSparkMax (ShooterConstants.kLeftFeedMotor,MotorType.kBrushed);
  private final CANSparkMax m_rightFeedMotor = new CANSparkMax (ShooterConstants.kRightFeedMotor,MotorType.kBrushed);

  private final CANSparkMax m_leftShootingMotor = new CANSparkMax (ShooterConstants.kLeftShootingMotor,MotorType.kBrushed);
  private final CANSparkMax m_rightShootingMotor = new CANSparkMax (ShooterConstants.kRightShootingMotor,MotorType.kBrushed);

  //declare sensor
  private final DigitalInput m_loadedSensor=new DigitalInput(ShooterConstants.kLoadedSensor);
  //declare angle
          //motor
          //sensor

  //Shuffleboard
  private final ShuffleboardTab shooterTab = Shuffleboard.getTab("Shooter");
  private GenericEntry loadedEntry;

  /** Creates a new Shooter. */
  public Shooter() {

    m_leftFeedMotor.restoreFactoryDefaults();
    m_rightFeedMotor.restoreFactoryDefaults();
    m_leftShootingMotor.restoreFactoryDefaults();
    m_rightShootingMotor.restoreFactoryDefaults();

    m_leftFeedMotor.setIdleMode(IdleMode.kBrake);
    m_rightFeedMotor.setIdleMode(IdleMode.kBrake);
    m_leftShootingMotor.setIdleMode(IdleMode.kBrake);
    m_rightShootingMotor.setIdleMode(IdleMode.kBrake);

    m_leftFeedMotor.setInverted(ShooterConstants.kLeftFeedMotorReversed);
    m_rightFeedMotor.setInverted(ShooterConstants.kRightFeedMotorReversed);
    m_leftShootingMotor.setInverted(ShooterConstants.kLeftShootingMotorReversed);
    m_rightShootingMotor.setInverted(ShooterConstants.kRightShootingMotorReversed);

    //Shuffleboard
    loadedEntry = shooterTab.add("Shooter Loaded",0).withWidget("Boolean Box").withSize(1,1).getEntry();
    

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    showShooterTelemetry();
  }


  public void showShooterTelemetry(){
    loadedEntry.setBoolean(m_loadedSensor.get());
  }
}
