package frc.utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class SwerveUtils {

    /**
     * Limit the linear and angular accelerations of the chassis between setpoints.
     * @param target - the target chassis velocity for the next period
     * @param previous - the previous target chassis velocity
     * @param dt - the time interval between periods
     * @param linear_acc_lim - the maximum linear acceleration allowed
     * @param rotational_acc_lim - the maximum rotational acceleration allowed
     */
    public static void RateLimitVelocity(ChassisSpeeds target, ChassisSpeeds previous, double dt, double linear_acc_lim, double rotational_acc_lim) {
        final double
            dvlim = linear_acc_lim * dt,        // the maximum change in velocity allowed for the given time interval
            drlim = rotational_acc_lim * dt,
            dvx = target.vxMetersPerSecond - previous.vxMetersPerSecond,    // the current change in velocity (components)
            dvy = target.vyMetersPerSecond - previous.vyMetersPerSecond,
            dvr = target.omegaRadiansPerSecond - previous.omegaRadiansPerSecond,
            dv = Math.hypot(dvx, dvy),                  // the current change in velocity (vector magnitude)
            _dv = MathUtil.clamp(dv, -dvlim, dvlim),    // the clamped magnitude
            _dr = MathUtil.clamp(dvr, -drlim, drlim),
            scale = dv == 0.0 ? 1.0 : _dv / dv,         // protect against div by 0 when delta velocity was (0, 0)
            _dvx = dvx * scale,                         // rescale component deltas based on clamped magnitude
            _dvy = dvy * scale;
        target.vxMetersPerSecond = previous.vxMetersPerSecond + _dvx;       // reapply clamped changes in velocity
        target.vyMetersPerSecond = previous.vyMetersPerSecond + _dvy;
        target.omegaRadiansPerSecond = previous.omegaRadiansPerSecond + _dr;
    }

}