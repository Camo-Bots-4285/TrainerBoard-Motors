package frc.lib.io.absoluteencoder;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class AbsoluteEncoderIODutyCycle implements AbsoluteEncoderIO {
    private final DutyCycleEncoder encoder;

    public AbsoluteEncoderIODutyCycle(int dioChannel) {
        encoder = new DutyCycleEncoder(dioChannel);
    }

    @Override
    public void updateInputs(AbsoluteEncoderInputs inputs) {
        inputs.connected = encoder.isConnected();

        // Returns rotations in [0,1)
        inputs.angle = Units.Rotations.of(encoder.get());
    }

    @Override
    public void close() {
        encoder.close();
    }
}