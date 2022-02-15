package com.swervedrivespecialties.swervelib.ctre;

import com.swervedrivespecialties.swervelib.AbsoluteEncoder;
import com.swervedrivespecialties.swervelib.AbsoluteEncoderFactory;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class MagEncoderFactoryBuilder {
    private Direction direction = Direction.COUNTER_CLOCKWISE;
    private int periodMilliseconds = 10;

    public MagEncoderFactoryBuilder withReadingUpdatePeriod(int periodMilliseconds) {
        this.periodMilliseconds = periodMilliseconds;
        return this;
    }

    public MagEncoderFactoryBuilder withDirection(Direction direction) {
        this.direction = direction;
        return this;
    }

    public AbsoluteEncoderFactory<MagEncoderAbsoluteConfiguration> build() {
        return configuration -> {
            DutyCycleEncoder encoder = new DutyCycleEncoder(configuration.getId());

            return new EncoderImplementation(encoder);
        };
    }

    private static class EncoderImplementation implements AbsoluteEncoder {
        private final DutyCycleEncoder encoder;

        private EncoderImplementation(DutyCycleEncoder encoder) {
            this.encoder = encoder;
        }

        @Override
        public double getAbsoluteAngle() {
            if(encoder.get() > 1 || encoder.get() < 0) {
                encoder.reset(); // clamp it to 1 at max and 0 at min
            }
            double angle = (encoder.get() - encoder.getPositionOffset()) * Math.PI * 2; // Absolute value from 0-1 mapped from 0-2pi
            angle %= 2.0 * Math.PI;
            if (angle < 0.0) {
                angle += 2.0 * Math.PI;
            }

            return angle;
        }
    }

    public enum Direction {
        CLOCKWISE,
        COUNTER_CLOCKWISE
    }
}
