package frc.lib.io.absoluteencoder;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase;


/**
 * REV Through Bore Encoder (TBE v1) using absolute duty-cycle mode.
 * https://docs.revrobotics.com/rev-crossover-products/sensors/tbe/v1/application-examples#brushless-motors
 */
public class AbsoluteEncoderIOSpark implements AbsoluteEncoderIO {
    private final AbsoluteEncoder encoder;

    public AbsoluteEncoderIOSpark(SparkBase spark) {
        this.encoder = spark.getAbsoluteEncoder();
    }

    @Override
    public void updateInputs(AbsoluteEncoderInputs inputs) {
        inputs.connected = true; // see note below
        inputs.angle = Rotations.of(encoder.getPosition());
    }
}