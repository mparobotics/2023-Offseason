package frc.lib.Config;

import java.net.CacheRequest;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import frc.robot.Constants;

public final class CTREConfigs {
    public CANCoderConfiguration swerveCanCoderConfiguration;

    public CTREConfigs() {
        swerveCanCoderConfiguration = new CANCoderConfiguration();

        /* Swerve CANCoder Configuration */
        swerveCanCoderConfiguration.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        swerveCanCoderConfiguration.sensorDirection = Constants.SwerveConstants.canCoderInvert;
        swerveCanCoderConfiguration.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        swerveCanCoderConfiguration.sensorTimeBase = SensorTimeBase.PerSecond;
    }
}
