// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import java.util.Optional;

import org.photonvision.PhotonCamera;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Robot extends TimedRobot {
  private static final int kFrontLeftChannel = 2;
  private static final int kRearLeftChannel = 4;
  private static final int kFrontRightChannel = 3;
  private static final int kRearRightChannel = 5;


  private static final int kIntakeSpin = 0;

  private MecanumDrive m_robotDrive;
  private SlewRateLimiter xLimiter = new SlewRateLimiter(1.0 / 0.4); // 1 / x seconds to 100%
  private SlewRateLimiter yLimiter = new SlewRateLimiter(1.0 / 0.4);
  private SlewRateLimiter spinLimiter = new SlewRateLimiter(1.0 / 0.4);
  private SlewRateLimiter intakeLimiter = new SlewRateLimiter(1.0 / 0.4);
   private SlewRateLimiter outputLimiter = new SlewRateLimiter(1.0 / 0.4);

  private final XboxController m_controller = new XboxController(0);
  // goofy ah gyro
  private final AHRS navX = new AHRS(SerialPort.Port.kUSB);
  final static double kCollisionThreshold_DeltaG = 0.4f;
  double last_world_linear_accel_x;
  double last_world_linear_accel_y;
  double lift;
  double drive;
  double drivespin;
  double shoot;
  ShuffleboardTab dash = Shuffleboard.getTab("SmartDashboard");

  PIDController linear = new PIDController(0.1, 0, 0);
  PIDController angular = new PIDController(0.05, 0, 0);
  PhotonCamera camera = new PhotonCamera("Arducam_USB_Camera");

  CANSparkMax intakeSpin = new CANSparkMax(20, MotorType.kBrushless);
  CANSparkMax armRotate = new CANSparkMax(1, MotorType.kBrushless);
  private RelativeEncoder armEncoder;
  PIDController arm = new PIDController(0.1, 0, 0);
  CANSparkMax test = new CANSparkMax(21, MotorType.kBrushless);

  @Override
  public void robotInit() {
    CANSparkMax frontLeft = new CANSparkMax(kFrontLeftChannel, MotorType.kBrushless);
    CANSparkMax rearLeft = new CANSparkMax(kRearLeftChannel, MotorType.kBrushless);
    CANSparkMax frontRight = new CANSparkMax(kFrontRightChannel, MotorType.kBrushless);
    CANSparkMax rearRight = new CANSparkMax(kRearRightChannel, MotorType.kBrushless);

    armEncoder = armRotate.getEncoder();
    //armEncoder = armRotate.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 8192);

    frontRight.setInverted(true);
    rearRight.setInverted(true);

    m_robotDrive = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);
  }

  @Override
  public void teleopPeriodic() {

    Optional<Alliance> ally = DriverStation.getAlliance();
if (ally.isPresent()) {
    if (ally.get() == Alliance.Red) {
        lift = .1;
        drive = .2;
        drivespin = .2;
        shoot = .25;
    }
    if (ally.get() == Alliance.Blue) {
        lift = .2;
        drive = .66;
        drivespin = .66;
        shoot = 1;
    }
}
else {
    //<NO COLOR YET ACTION>
}

    int dPad = m_controller.getPOV();

  if (dPad == 0) {
    //armRotate.set(intakeLimiter.calculate(0.2));
    armRotate.set(lift * arm.calculate(armEncoder.getPosition(), 0));
  } else if (dPad == 180) {
    //armRotate.set(intakeLimiter.calculate(-0.2));
    armRotate.set(lift * arm.calculate(armEncoder.getPosition(), -36));
  } else if (dPad == 90) {
    //armRotate.set(intakeLimiter.calculate(-0.2));
    armRotate.set(lift * arm.calculate(armEncoder.getPosition(), -22));
  } else if (dPad == 270) {
    //armRotate.set(intakeLimiter.calculate(-0.2));
    armRotate.set(lift * arm.calculate(armEncoder.getPosition(), -26));
    
  }else {
    armRotate.set(0);
  }

  
  if (m_controller.getRightTriggerAxis() > 0.1) {
    intakeSpin.set(intakeLimiter.calculate(0.33));
  } else if (m_controller.getLeftTriggerAxis() > 0.1) {
    intakeSpin.set(intakeLimiter.calculate(-0.33));
  }else {
    intakeSpin.set(0);
  }
  if (m_controller.getLeftBumper()) {
    test.set(outputLimiter.calculate(-shoot));
  } else {
    test.set(outputLimiter.calculate(0));
  }

  SmartDashboard.putNumber("Encoder Position", armEncoder.getPosition());

    double x = (m_controller.getLeftX() * Math.abs(m_controller.getLeftX()) * drive);
    x = xLimiter.calculate(x);
    double y = (m_controller.getLeftY() * Math.abs(m_controller.getLeftY()) * -drive);
    y = yLimiter.calculate(y);
    double spin = (m_controller.getRightX() * Math.abs(m_controller.getRightX()) * drivespin);
    spin = spinLimiter.calculate(spin);
    //these values come from the joystick

    double finalAngle;
    double finalX;
    double finalY;
    double initAngle = Math.atan(y/x);

    if (x < 0) {
      initAngle += Math.PI;
    } else if (y < 0) {
      initAngle += 2*Math.PI;
    };

    double hypo = Math.abs(Math.sqrt((x * x) + (y * y)));
    double gyro = navX.getYaw() * (Math.PI/180);

    finalAngle = (initAngle + gyro);
    finalX = (Math.cos(finalAngle) * hypo);
    finalY = (Math.sin(finalAngle) * hypo);


    /*if (m_controller.getBButton()) {
      // Vision-alignment mode
      // Query the latest result from PhotonVision
      var result = camera.getLatestResult();
      intakeSpin.set(intakeLimiter.calculate(0.33));

      if (result.hasTargets()) {
          // First calculate range
          double range =
                  PhotonUtils.calculateDistanceToTargetMeters(
                          0.29,
                          0.025,
                          0,
                          Units.degreesToRadians(result.getBestTarget().getPitch()));

          // Use this range as the measurement we give to the PID controller.
          // -1.0 required to ensure positive PID controller effort _increases_ range
          finalY = 0.25 + (-0.8 * linear.calculate(range, 0));
          spin = -0.25 * angular.calculate(result.getBestTarget().getYaw(), 0);
        
      } else {
        finalY = 0;
        spin = 0;
        armRotate.set(0.15 * arm.calculate(armEncoder.getPosition(), -22));
      }
    }*/

    m_robotDrive.driveCartesian(finalY, finalX, spin);

    double rightRumbleStrength = 0;
    double leftRumbleStrength = 0;
    double rumbleStrength = hypo + Math.abs(spin); // = Math.abs(Math.sqrt(Math.pow(m_controller.getLeftX, 2) + Math.pow(m_controller.getLeftY, 2))) + Math.abs(m_controller.getRightX);

    if (rumbleStrength < 0.25) {
      leftRumbleStrength = (hypo + Math.abs(spin * 0.66)); // Math.abs(Math.sqrt(Math.pow(m_controller.getLeftX, 2) + Math.pow(m_controller.getLeftY, 2))) + (0.66 * Math.abs(m_controller.getRightX));
    } else if (rumbleStrength < 0.5) {
      rightRumbleStrength = 0.5 * (hypo + Math.abs(spin * 0.5)); // Math.abs(Math.sqrt(Math.pow(m_controller.getLeftX, 2) + Math.pow(m_controller.getLeftY, 2))) + (0.5 * Math.abs(m_controller.getRightX));
      leftRumbleStrength = (0.5 - hypo + Math.abs(spin * 0.5)) ; // Math.abs(Math.sqrt(Math.pow(m_controller.getLeftX, 2) + Math.pow(m_controller.getLeftY, 2))) + (0.5 * Math.abs(m_controller.getRightX));
    } else {
      rightRumbleStrength = 0.5 * (hypo + Math.abs(spin * 0.5)); // Math.abs(Math.sqrt(Math.pow(m_controller.getLeftX, 2) + Math.pow(m_controller.getLeftY, 2))) + (0.5 * Math.abs(m_controller.getRightX));
      leftRumbleStrength = 0.66 * Math.abs(spin); // m_controller.getRightX
    }

    m_controller.setRumble(RumbleType.kLeftRumble, leftRumbleStrength);
    m_controller.setRumble(RumbleType.kRightRumble, rightRumbleStrength);

      /* 
  public boolean detectCollision() {
    double curr_world_linear_accel_x = navX.getWorldLinearAccelX();
    double currentJerkX = curr_world_linear_accel_x - last_world_linear_accel_x;
    last_world_linear_accel_x = curr_world_linear_accel_x;
    double curr_world_linear_accel_y = navX.getWorldLinearAccelY();
    double currentJerkY = curr_world_linear_accel_y - last_world_linear_accel_y;
    last_world_linear_accel_y = curr_world_linear_accel_y;

    return ((Math.abs(currentJerkX) > kCollisionThreshold_DeltaG) ||
      (Math.abs(currentJerkY) > kCollisionThreshold_DeltaG));
  } */
    
  }
}
