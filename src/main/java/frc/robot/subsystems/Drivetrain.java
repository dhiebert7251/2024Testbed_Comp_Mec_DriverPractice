// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//import com.revrobotics.CANSparkMax;
//import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
//import edu.wpi.first.math.kinematics.MecanumDriveMotorVoltages;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PhysicalConstants;
import frc.robot.Constants.AutoConstants;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;
//import com.revrobotics.RelativeEncoder;
import com.pathplanner.lib.auto.AutoBuilder;



public class Drivetrain extends SubsystemBase {
  //Compbot motor controllers
  //private final CANSparkMax m_frontLeft = new CANSparkMax(DriveConstants.kFrontLeftMotorPort, MotorType.kBrushless);
  //private final CANSparkMax m_rearLeft = new CANSparkMax(DriveConstants.kRearLeftMotorPort, MotorType.kBrushless);
  //private final CANSparkMax m_frontRight = new CANSparkMax(DriveConstants.kFrontRightMotorPort, MotorType.kBrushless);
  //private final CANSparkMax m_rearRight = new CANSparkMax(DriveConstants.kRearRightMotorPort, MotorType.kBrushless);

  //private MecanumDrive m_drive = new MecanumDrive(m_frontLeft, m_rearLeft, m_frontRight, m_rearRight);


  //Testbed motor controllers
  private final WPI_TalonSRX m_frontLeft = new WPI_TalonSRX(DriveConstants.kFrontLeftMotorPort);
  private final WPI_VictorSPX m_rearLeft = new WPI_VictorSPX(DriveConstants.kRearLeftMotorPort);
  private final WPI_TalonSRX m_frontRight = new WPI_TalonSRX(DriveConstants.kFrontRightMotorPort);
  private final WPI_VictorSPX m_rearRight = new WPI_VictorSPX(DriveConstants.kRearRightMotorPort);

  private final MecanumDrive m_drive =
    new MecanumDrive(m_frontLeft::set, m_rearLeft::set, m_frontRight::set, m_rearRight::set);

  private Field2d field = new Field2d();


  //Encoders
  
    // Compbot encoders
    //private final RelativeEncoder m_frontLeftEncoder; 
    //private final RelativeEncoder m_rearLeftEncoder; 
    //private final RelativeEncoder m_frontRightEncoder; 
    //private final RelativeEncoder m_rearRightEncoder;  

  // The front-left-side drive encoder
  private final Encoder m_frontLeftEncoder =
      new Encoder(
          DriveConstants.kFrontLeftEncoderPorts[0],
          DriveConstants.kFrontLeftEncoderPorts[1],
          DriveConstants.kFrontLeftEncoderReversed);

  // The rear-left-side drive encoder
  private final Encoder m_rearLeftEncoder =
      new Encoder(
          DriveConstants.kRearLeftEncoderPorts[0],
          DriveConstants.kRearLeftEncoderPorts[1],
          DriveConstants.kRearLeftEncoderReversed);

  // The front-right--side drive encoder
  private final Encoder m_frontRightEncoder =
      new Encoder(
          DriveConstants.kFrontRightEncoderPorts[0],
          DriveConstants.kFrontRightEncoderPorts[1],
          DriveConstants.kFrontRightEncoderReversed);

  // The rear-right-side drive encoder
  private final Encoder m_rearRightEncoder =
      new Encoder(
          DriveConstants.kRearRightEncoderPorts[0],
          DriveConstants.kRearRightEncoderPorts[1],
          DriveConstants.kRearRightEncoderReversed);

  // Wheel PID controllers
  private final PIDController m_frontLeftPIDController =
      new PIDController(PhysicalConstants.kPFrontLeft, 
                        PhysicalConstants.kIFrontLeft,
                        PhysicalConstants.kDFrontLeft);

  private final PIDController m_rearLeftPIDController = 
      new PIDController(PhysicalConstants.kPRearLeft, 
                        PhysicalConstants.kIRearLeft,
                        PhysicalConstants.kDRearLeft);

  private final PIDController m_frontRightPIDController =
      new PIDController(PhysicalConstants.kPFrontRight, 
                        PhysicalConstants.kIFrontRight,
                        PhysicalConstants.kDFrontRight);

  private final PIDController m_rearRightPIDController =
      new PIDController(PhysicalConstants.kPRearRight, 
                        PhysicalConstants.kIRearRight,
                        PhysicalConstants.kDRearRight);

  // The gyro sensor
  private final AHRS m_gyro = new AHRS();



  // Odometry class for tracking robot pose
  MecanumDriveOdometry m_odometry =
      new MecanumDriveOdometry(
          PhysicalConstants.kDriveKinematics,
          m_gyro.getRotation2d(),
          new MecanumDriveWheelPositions());

  // The feedforward for the drive TODO: how do you tune this?
  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1, 3);

  /** Creates a new DriveSubsystem. */
  public Drivetrain() {
    // Calibrate gyro
      new Thread(() -> {
        try {
          Thread.sleep(1000);
          zeroHeading();
        } catch (Exception e){
        }
      }).start();


    //Register values for dashboard
    SendableRegistry.addChild(m_drive, m_frontLeft);
    SendableRegistry.addChild(m_drive, m_rearLeft);
    SendableRegistry.addChild(m_drive, m_frontRight);
    SendableRegistry.addChild(m_drive, m_rearRight);
    //TODO: add other sensors? (encoders, gyro, etc.)

    //Compbot encoders
    //m_frontLeftEncoder = m_frontLeft.getEncoder();
    //m_rearLeftEncoder = m_rearLeft.getEncoder();
    //m_frontRightEncoder = m_frontRight.getEncoder();
    //m_rearRightEncoder = m_rearRight.getEncoder();

    //Compbot Factory reset motor controllers 
    //m_frontLeft.restoreFactoryDefaults();
    //m_rearLeft.restoreFactoryDefaults();
    //m_frontRight.restoreFactoryDefaults();
    //m_rearRight.restoreFactoryDefaults();

    //Testbed Factory reset motor controllers
    m_frontLeft.configFactoryDefault();
    m_rearLeft.configFactoryDefault();
    m_frontRight.configFactoryDefault();
    m_rearRight.configFactoryDefault();

    //Compbot break/coast mode
    //m_frontLeft.setIdleMode(IdleMode.kCoast);
    //m_rearLeft.setIdleMode(IdleMode.kCoast);
    //m_frontRight.setIdleMode(IdleMode.kCoast);
    //m_rearRight.setIdleMode(IdleMode.kCoast);

    //Testbed break/coast mode
    m_frontLeft.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Coast);
    m_rearLeft.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Coast);
    m_frontRight.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Coast);
    m_rearRight.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Coast);

    // Sets the distance per pulse for the encoders
    //Compbot
    //m_frontLeftEncoder.setPositionConversionFactor(PhysicalConstants.kEncoderDistancePerPulse);
    //m_rearLeftEncoder.setPositionConversionFactor(PhysicalConstants.kEncoderDistancePerPulse);
    //m_frontRightEncoder.setPositionConversionFactor(PhysicalConstants.kEncoderDistancePerPulse);
    //m_rearRightEncoder.setPositionConversionFactor(PhysicalConstants.kEncoderDistancePerPulse);

    //Testbed
    m_frontLeftEncoder.setDistancePerPulse(PhysicalConstants.kEncoderDistancePerPulse);
    m_rearLeftEncoder.setDistancePerPulse(PhysicalConstants.kEncoderDistancePerPulse);
    m_frontRightEncoder.setDistancePerPulse(PhysicalConstants.kEncoderDistancePerPulse);
    m_rearRightEncoder.setDistancePerPulse(PhysicalConstants.kEncoderDistancePerPulse);
    
    //Invert motors on one side
    m_frontLeft.setInverted(DriveConstants.kFrontLeftMotorReversed);
    m_rearLeft.setInverted(DriveConstants.kRearLeftMotorReversed);
    m_frontRight.setInverted(DriveConstants.kFrontRightMotorReversed);
    m_rearRight.setInverted(DriveConstants.kRearRightMotorReversed);


    
    // Configure AutoBuilder
    AutoBuilder.configureHolonomic(
      this::getPose, 
      this::resetPose, 
      this::getChassisSpeeds, 
      this::driveFieldRelative, 
      AutoConstants.kPathFollowerConfig,
      () -> {
        // Boolean supplier that controls when the path will be mirrored for the red alliance
        // This will flip the path being followed to the red side of the field.
        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        }

        return false;
      },
      this
    );
    

    
  }

  @Override
  public void periodic() {

    // Update the odometry in the periodic block
    m_odometry.update(m_gyro.getRotation2d(), getCurrentWheelDistances());

    // Update the field
    field.setRobotPose(getPose());

    showTelemetry();
  }



  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  //TODO: update for aprilTag detection of current pose

  public void resetPose(Pose2d pose) {
    m_odometry.resetPosition(m_gyro.getRotation2d(), getCurrentWheelDistances(), pose);
  }

 

  public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
    driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getPose().getRotation()));
  }

  
  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

    MecanumDriveWheelSpeeds targetStates = PhysicalConstants.kDriveKinematics.toWheelSpeeds(targetSpeeds);
    setSpeeds(targetStates);

  }



  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(m_gyro.getRotation2d(), getCurrentWheelDistances(), pose);
  }

/**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(
    double xSpeed, double ySpeed, double rot, boolean fieldRelative, double periodSeconds) {
      var mecanumDriveWheelSpeeds =
          PhysicalConstants.kDriveKinematics.toWheelSpeeds(
             ChassisSpeeds.discretize(
                  fieldRelative
                      ? ChassisSpeeds.fromFieldRelativeSpeeds(
                          xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                      : new ChassisSpeeds(xSpeed, ySpeed, rot),
                  periodSeconds));
      mecanumDriveWheelSpeeds.desaturate(PhysicalConstants.kMaxVelocity);
      setSpeeds(mecanumDriveWheelSpeeds);

      SmartDashboard.putNumber("Joy1 X", xSpeed);
      SmartDashboard.putNumber("Joy1 Y", ySpeed);
      SmartDashboard.putNumber("Joy2 X", rot);

  }



  /* Sets the front left drive MotorController to a voltage. */
  /*
  public void setDriveMotorControllersVolts(MecanumDriveMotorVoltages volts) {
    m_frontLeft.setVoltage(volts.frontLeftVoltage);
    m_rearLeft.setVoltage(volts.rearLeftVoltage);
    m_frontRight.setVoltage(volts.frontRightVoltage);
    m_rearRight.setVoltage(volts.rearRightVoltage);
  }
*/
  

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    //Compbot
    //m_frontLeftEncoder.setPosition(0);
    //m_rearLeftEncoder.setPosition(0);
    //m_frontRightEncoder.setPosition(0);
    //m_rearRightEncoder.setPosition(0);

    //Testbed
    m_frontLeftEncoder.reset();
    m_rearLeftEncoder.reset();
    m_frontRightEncoder.reset();
    m_rearRightEncoder.reset();
  }

  /**
   * Gets the front left drive encoder.
   *
   * @return the front left drive encoder
   */
  public Encoder getFrontLeftEncoder() {
    //Compbot
    //return m_frontLeftEncoder.getEncoder();

    //Testbed
    return m_frontLeftEncoder;
  }

  /**
   * Gets the rear left drive encoder.
   *
   * @return the rear left drive encoder
   */
  public Encoder getRearLeftEncoder() {
    //Compbot
    //return m_rearLeftEncoder.getEncoder();

    //Testbed
    return m_rearLeftEncoder;
  }

  /**
   * Gets the front right drive encoder.
   *
   * @return the front right drive encoder
   */
  public Encoder getFrontRightEncoder() {
    //Compbot
    //return m_frontRightEncoder.getEncoder();

    //Testbed
    return m_frontRightEncoder;
  }

  /**
   * Gets the rear right drive encoder.
   *
   * @return the rear right encoder
   */
  public Encoder getRearRightEncoder() {
    //Compbot
    //return m_rearRightEncoder.getEncoder();

    //Testbed
    return m_rearRightEncoder;
  }

  /**
   * Gets the current wheel speeds.
   *
   * @return the current wheel speeds in a MecanumDriveWheelSpeeds object.
   */
  public MecanumDriveWheelSpeeds getCurrentWheelSpeeds() {
    SmartDashboard.putNumber("FL Encoder Rate", m_frontLeftEncoder.getRate());
    SmartDashboard.putNumber("RL Encoder Rate", m_rearLeftEncoder.getRate());
    SmartDashboard.putNumber("FR Encoder Rate", m_frontRightEncoder.getRate());
    SmartDashboard.putNumber("RR Encoder Rate", m_rearRightEncoder.getRate());

    return new MecanumDriveWheelSpeeds(

        //Compbot
        //m_frontLeftEncoder.getVelocity(),
        //m_rearLeftEncoder.getVelocity(),
        //m_frontRightEncoder.getVelocity(),
        //m_rearRightEncoder.getVelocity());

        //Testbed
        m_frontLeftEncoder.getRate(),
        m_rearLeftEncoder.getRate(),
        m_frontRightEncoder.getRate(),
        m_rearRightEncoder.getRate());


  }

  /**
   * Gets the current wheel distance measurements.
   *
   * @return the current wheel distance measurements in a MecanumDriveWheelPositions object.
   */
  public MecanumDriveWheelPositions getCurrentWheelDistances() {
    SmartDashboard.putNumber("FL Encoder Distance", m_frontLeftEncoder.getDistance());
    SmartDashboard.putNumber("RL Encoder Distance", m_rearLeftEncoder.getDistance());
    SmartDashboard.putNumber("FR Encoder Distance", m_frontRightEncoder.getDistance());
    SmartDashboard.putNumber("RR Encoder Distance", m_rearRightEncoder.getDistance());
    //Compbot
    //return new MecanumDriveWheelPositions(
    //    m_frontLeftEncoder.getPosition(),
    //    m_rearLeftEncoder.getPosition(),
    //    m_frontRightEncoder.getPosition(),
    //    m_rearRightEncoder.getPosition());  


    //Testbed
    return new MecanumDriveWheelPositions(
        m_frontLeftEncoder.getDistance(),
        m_rearLeftEncoder.getDistance(),
        m_frontRightEncoder.getDistance(),
        m_rearRightEncoder.getDistance());
  }

  /**
   * Set the desired speeds for each wheel.
   *
   * @param speeds The desired wheel speeds.
   */
  public void setSpeeds(MecanumDriveWheelSpeeds speeds) {
    final double frontLeftFeedforward = m_feedforward.calculate(speeds.frontLeftMetersPerSecond);
    final double frontRightFeedforward = m_feedforward.calculate(speeds.frontRightMetersPerSecond);
    final double backLeftFeedforward = m_feedforward.calculate(speeds.rearLeftMetersPerSecond);
    final double backRightFeedforward = m_feedforward.calculate(speeds.rearRightMetersPerSecond);

    final double frontLeftOutput =
        m_frontLeftPIDController.calculate(
            m_frontLeftEncoder.getRate(), speeds.frontLeftMetersPerSecond);
    final double frontRightOutput =
        m_frontRightPIDController.calculate(
            m_frontRightEncoder.getRate(), speeds.frontRightMetersPerSecond);
    final double backLeftOutput =
        m_rearLeftPIDController.calculate(
            m_rearLeftEncoder.getRate(), speeds.rearLeftMetersPerSecond);
    final double backRightOutput =
        m_rearRightPIDController.calculate(
            m_rearRightEncoder.getRate(), speeds.rearRightMetersPerSecond);

    m_frontLeft.setVoltage(frontLeftOutput + frontLeftFeedforward);
    m_frontRight.setVoltage(frontRightOutput + frontRightFeedforward);
    m_rearLeft.setVoltage(backLeftOutput + backLeftFeedforward);
    m_rearRight.setVoltage(backRightOutput + backRightFeedforward);

    SmartDashboard.putNumber("FL Voltage", m_frontLeft.getMotorOutputVoltage());
    SmartDashboard.putNumber("RL Voltage", m_rearLeft.getMotorOutputVoltage());    
    SmartDashboard.putNumber("FR Voltage", m_frontRight.getMotorOutputVoltage());    
    SmartDashboard.putNumber("RR Voltage", m_rearRight.getMotorOutputVoltage());
  }

  public ChassisSpeeds getChassisSpeeds(){
    // Convert to chassis speeds
    ChassisSpeeds chassisSpeeds = PhysicalConstants.kDriveKinematics.toChassisSpeeds(getCurrentWheelSpeeds());
    return chassisSpeeds;
  }



  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -m_gyro.getRate();
  }

  public boolean getFieldRelative() {
    return m_gyro.isConnected();
  }


/*
    // Set up custom logging to add the current path to a field 2d widget
    PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));

    SmartDashboard.putData("Field", field);
    */

  public void showTelemetry(){
    SmartDashboard.putNumber("Heading", getHeading());
    SmartDashboard.putNumber("Turn Rate", getTurnRate());
    SmartDashboard.putBoolean("Field Relative", getFieldRelative());

    SmartDashboard.putNumber("FL Encoder Position", m_frontLeftEncoder.getRaw());
    SmartDashboard.putNumber("RL Encoder Position", m_rearLeftEncoder.getRaw());
    SmartDashboard.putNumber("FR Encoder Position", m_frontRightEncoder.getRaw());
    SmartDashboard.putNumber("RR Encoder Position", m_rearRightEncoder.getRaw());
    
  }
}
