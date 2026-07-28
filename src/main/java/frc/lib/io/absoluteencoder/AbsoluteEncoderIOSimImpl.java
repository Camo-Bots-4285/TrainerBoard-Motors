package frc.lib.io.absoluteencoder;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;

public class AbsoluteEncoderIOSimImpl implements AbsoluteEncoderIOSim {

    private Angle angle = Rotations.of(0);

    @Override
    public void updateInputs(AbsoluteEncoderInputs inputs) {
        inputs.connected = true;
        inputs.angle = angle;
    }

    @Override
    public void setAngle(Angle angle) {
        this.angle = angle;
    }
}