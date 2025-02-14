package com.swervedrivespecialties.swervelib.ctre;

public class MagEncoderAbsoluteConfiguration {
    private final int id;
    private final double offset;

    public MagEncoderAbsoluteConfiguration(int id, double offset) {
        this.id = id;
        this.offset = offset;
    }

    public int getId() {
        return id;
    }

    public double getOffset() {
        return offset;
    }
}
