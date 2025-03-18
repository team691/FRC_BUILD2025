package frc.robot.Constants;

import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.*;


import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.GyroSimulation;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.Constants.ModuleConstants;

public final class Configs {
        public static final class DriveTrainConstants{
                public static final double WHEEL_COEFFICIENT_OF_FRICTION = 1.5;

        public static final Mass ROBOT_MASS = Kilograms.of(24.9476); // robot weight with bumpers

        /** TODO: change motor type to match your robot */
        public static final DCMotor DRIVE_MOTOR_MODEL = DCMotor.getNEO(1);

        public static final DCMotor STEER_MOTOR_MODEL = DCMotor.getNeo550(1);

        /** numbers imported from {@link TunerConstants} TODO: for REV chassis, replace them with actual numbers */
        public static final Distance WHEEL_RADIUS = 
        Meters.of(2);

        public static final double DRIVE_GEAR_RATIO = 0.410;
        public static final double STEER_GEAR_RATIO = 0;

        public static final Voltage STEER_FRICTION_VOLTAGE = Volts.of(12);
        public static final Voltage DRIVE_FRICTION_VOLTAGE = Volts.of(12);
        public static final MomentOfInertia STEER_INERTIA = KilogramSquareMeters.of(0.025);

        /* adjust current limit */
        public static final Current DRIVE_ANTI_SLIP_TORQUE_CURRENT_LIMIT = Amps.of(80);
        public static final Current DRIVE_OVER_CURRENT_PROTECTION = Amps.of(65);
        public static final Time DRIVE_OVERHEAT_PROTECTION_TIME = Seconds.of(1);
        public static final Current DRIVE_OVERHEAT_PROTECTION_CURRENT = Amps.of(45);
        public static final Current STEER_CURRENT_LIMIT = Amps.of(15);

        public static final Current OVER_CURRENT_WARNING = Amps.of(240);

        /** translations of the modules to the robot center, in FL, FR, BL, BR */
        static double value = Meters.convertFrom(0.33, Inch);
        public static final Translation2d[] MODULE_TRANSLATIONS = new Translation2d[] {
                new Translation2d(value,value),
                new Translation2d(value, -value),
                new Translation2d(-value, value),
                new Translation2d(-value, -value)
        };

        public static final Distance TRACK_LENGTH =
                MODULE_TRANSLATIONS[0].minus(MODULE_TRANSLATIONS[3]).getMeasureX();
        public static final Distance TRACK_WIDTH =
                MODULE_TRANSLATIONS[0].minus(MODULE_TRANSLATIONS[3]).getMeasureY();

        /* equations that calculates some constants for the simulator (don't modify) */
        private static final double GRAVITY_CONSTANT = 9.8;

        public static final Distance DRIVE_BASE_RADIUS = Meters.of(MODULE_TRANSLATIONS[0].getNorm());

        /* friction_force = normal_force * coefficient_of_friction */
        public static final LinearAcceleration MAX_FRICTION_ACCELERATION =
                MetersPerSecondPerSecond.of(GRAVITY_CONSTANT * WHEEL_COEFFICIENT_OF_FRICTION);

        /* force = torque / distance */
        public static final Force MAX_PROPELLING_FORCE = NewtonMeters.of(
                        DRIVE_MOTOR_MODEL.getTorque(DRIVE_ANTI_SLIP_TORQUE_CURRENT_LIMIT.in(Amps)) * DRIVE_GEAR_RATIO)
                .div(WHEEL_RADIUS)
                .times(4);

        /* floor_speed = wheel_angular_velocity * wheel_radius */
        public static final LinearVelocity CHASSIS_MAX_VELOCITY = MetersPerSecond.of(DRIVE_MOTOR_MODEL.getSpeed(
                        DRIVE_MOTOR_MODEL.getTorque(
                                DRIVE_MOTOR_MODEL.getCurrent(0, 12)),
                        12)
                / DRIVE_GEAR_RATIO
                * WHEEL_RADIUS.in(Meters));
        public static final LinearAcceleration CHASSIS_MAX_ACCELERATION =
                (LinearAcceleration) Measure.min(MAX_FRICTION_ACCELERATION, MAX_PROPELLING_FORCE.div(ROBOT_MASS));
        public static final AngularVelocity CHASSIS_MAX_ANGULAR_VELOCITY =
                RadiansPerSecond.of(CHASSIS_MAX_VELOCITY.in(MetersPerSecond) / DRIVE_BASE_RADIUS.in(Meters));
        public static final AngularAcceleration CHASSIS_MAX_ANGULAR_ACCELERATION = RadiansPerSecondPerSecond.of(
                CHASSIS_MAX_ACCELERATION.in(MetersPerSecondPerSecond) / DRIVE_BASE_RADIUS.in(Meters) * 2);

        public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(MODULE_TRANSLATIONS);

        /* for collision detection in simulation */
        public static final Distance BUMPER_WIDTH = Inches.of(32), BUMPER_LENGTH = Inches.of(32);

        // https://unacademy.com/content/upsc/study-material/physics/moment-of-inertia-of-rectangle-section/
        public static final MomentOfInertia ROBOT_MOI = KilogramSquareMeters.of(ROBOT_MASS.in(Kilograms)
                * (BUMPER_WIDTH.in(Meters) * BUMPER_WIDTH.in(Meters) + BUMPER_LENGTH.in(Meters) * BUMPER_LENGTH.in(Meters))
                / 12.0);

        public static final Supplier<GyroSimulation> gyroSimulationFactory = COTS.ofNav2X();

        /* dead configs, don't change them */
        public static final int ODOMETRY_CACHE_CAPACITY = 10;
        public static final double ODOMETRY_FREQUENCY = 250.0;
        public static final double ODOMETRY_WAIT_TIMEOUT_SECONDS = 0.1;
        public static final int SIMULATION_TICKS_IN_1_PERIOD = 5;
        }
    public static final class MAXSwerveModule {
        public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
        public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

        static {
            // Use module constants to calculate conversion factors and feed forward gain.
            double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI
                    / ModuleConstants.kDrivingMotorReduction;
            double turningFactor = 2 * Math.PI;
            double drivingVelocityFeedForward = 1 / ModuleConstants.kDriveWheelFreeSpeedRps;

            drivingConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(50);
            drivingConfig.encoder
                    .positionConversionFactor(drivingFactor) // meters
                    .velocityConversionFactor(drivingFactor / 60.0); // meters per second
            drivingConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(0.04, 0, 0)
                    .velocityFF(drivingVelocityFeedForward)
                    .outputRange(-1, 1);

            turningConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(20);
            turningConfig.absoluteEncoder
                    // Invert the turning encoder, since the output shaft rotates in the opposite
                    // direction of the steering motor in the MAXSwerve Module.
                    .inverted(true)
                    .positionConversionFactor(turningFactor) // radians
                    .velocityConversionFactor(turningFactor / 60.0); // radians per second
            turningConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(0.5, 0, 0)
                    .outputRange(-1, 1)
                    // Enable PID wrap around for the turning motor. This will allow the PID
                    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
                    // to 10 degrees will go through 0 rather than the other direction which is a
                    // longer route.
                    .positionWrappingEnabled(true)
                    .positionWrappingInputRange(0, turningFactor);
        }
    }
}