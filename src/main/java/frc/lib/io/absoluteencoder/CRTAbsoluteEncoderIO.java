package frc.lib.io.absoluteencoder;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;

/**
 * An {@link AbsoluteEncoderIO} implementation that combines two absolute encoders mounted on
 * different gears driven by the same mechanism to create a multi-turn absolute encoder.
 *
 * <p>This implementation uses a continuous form of the Chinese Remainder Theorem (CRT). Rather
 * than treating each encoder as a discrete set of gear tooth positions, it uses the full angular
 * resolution of both encoders to reconstruct the mechanism's position over multiple rotations.
 *
 * <p>Example:
 *
 * <pre>
 * Main Gear:      40 teeth
 * Encoder A Gear: 37 teeth
 * Encoder B Gear: 39 teeth
 * </pre>
 *
 * <p>The encoders measure:
 *
 * <pre>
 * A = θ * (40 / 37) mod 1
 * B = θ * (40 / 39) mod 1
 * </pre>
 *
 * where θ is the mechanism angle in rotations.
 *
 * <p>The unique measurement range before the pattern repeats is:
 *
 * <pre>
 * lcm(37, 39) / 40 ≈ 36.075 rotations
 * </pre>
 *
 * allowing the mechanism position to be determined over approximately 36 rotations while
 * preserving the continuous precision of the underlying encoders.
 */
public class CRTAbsoluteEncoderIO implements AbsoluteEncoderIO {

    /** First encoder participating in the CRT reconstruction. */
    private final AbsoluteEncoderIO encoderA;

    /** Second encoder participating in the CRT reconstruction. */
    private final AbsoluteEncoderIO encoderB;

    /** Tooth count of the mechanism gear being measured. */
    private final int mainGearTeeth;

    /** Tooth count of the gear connected to encoder A. */
    private final int encoderAGearTeeth;

    /** Tooth count of the gear connected to encoder B. */
    private final int encoderBGearTeeth;

    /** Cached inputs for encoder A. */
    protected final AbsoluteEncoderInputsAutoLogged inputsA =
            new AbsoluteEncoderInputsAutoLogged();

    /** Cached inputs for encoder B. */
    protected final AbsoluteEncoderInputsAutoLogged inputsB =
            new AbsoluteEncoderInputsAutoLogged();

    /**
     * Maximum number of mechanism rotations that can be uniquely represented before the gear
     * pattern repeats.
     */
    private final double maxMainRotations;

    /**
     * Creates a CRT-based multi-turn absolute encoder.
     *
     * @param encoderA First absolute encoder.
     * @param encoderAGearTeeth Tooth count of the gear attached to encoder A.
     * @param encoderB Second absolute encoder.
     * @param encoderBGearTeeth Tooth count of the gear attached to encoder B.
     * @param mainGearTeeth Tooth count of the mechanism gear being measured.
     */
    public CRTAbsoluteEncoderIO(
            AbsoluteEncoderIO encoderA,
            int encoderAGearTeeth,
            AbsoluteEncoderIO encoderB,
            int encoderBGearTeeth,
            int mainGearTeeth) {

        this.encoderA = encoderA;
        this.encoderB = encoderB;

        this.encoderAGearTeeth = encoderAGearTeeth;
        this.encoderBGearTeeth = encoderBGearTeeth;
        this.mainGearTeeth = mainGearTeeth;

        // Number of mechanism rotations before the encoder pattern repeats.
        this.maxMainRotations =
                lcm(encoderAGearTeeth, encoderBGearTeeth)
                        / (double) mainGearTeeth;
    }

    /**
     * Updates the reconstructed mechanism angle.
     *
     * <p>The algorithm:
     *
     * <ol>
     *   <li>Reads both encoder angles.
     *   <li>Computes the gear ratio between the mechanism and each encoder.
     *   <li>Generates all possible unwrapped positions consistent with encoder A.
     *   <li>Predicts what encoder B should read for each candidate.
     *   <li>Selects the candidate with the smallest circular error relative to encoder B.
     * </ol>
     *
     * <p>The resulting angle is expressed in mechanism rotations and is unique within the
     * reconstruction range defined by {@link #maxMainRotations}.
     *
     * @param inputs Structure to populate with the reconstructed mechanism position.
     */
    @Override
    public void updateInputs(AbsoluteEncoderInputs inputs) {
        encoderA.updateInputs(inputsA);
        encoderB.updateInputs(inputsB);

        inputs.connected = inputsA.connected && inputsB.connected;

        if (!inputs.connected
                || inputsA.angle == null
                || inputsB.angle == null) {
            return;
        }

        // Encoder angles in rotations [0, 1).
        double a = inputsA.angle.in(Rotations);
        double b = inputsB.angle.in(Rotations);

        // Encoder rotations per mechanism rotation.
        double ratioA = (double) mainGearTeeth / encoderAGearTeeth;
        double ratioB = (double) mainGearTeeth / encoderBGearTeeth;

        double bestTheta = 0.0;
        double bestError = Double.POSITIVE_INFINITY;

        // Number of possible wraps of encoder A that could occur within
        // the valid reconstruction range.
        int wrapsToCheck = (int) Math.ceil(maxMainRotations * ratioA);

        for (int wrap = 0; wrap <= wrapsToCheck; wrap++) {

            // Treat encoder A as if it has completed additional whole turns.
            double encoderATotal = a + wrap;

            // Convert the candidate encoder position back to mechanism rotations.
            double theta = encoderATotal / ratioA;

            if (theta < 0 || theta >= maxMainRotations) {
                continue;
            }

            // Predict what encoder B should read for this mechanism angle.
            double predictedB = mod1(theta * ratioB);

            // Measure angular disagreement between prediction and measurement.
            double error = circularDistance(predictedB, b);

            if (error < bestError) {
                bestError = error;
                bestTheta = theta;
            }
        }

        inputs.angle = Rotations.of(bestTheta);
    }

    /** Closes both underlying encoders. */
    @Override
    public void close() {
        encoderA.close();
        encoderB.close();
    }

    /**
     * Computes a modulo operation over the range [0, 1).
     *
     * @param value Value to wrap.
     * @return Equivalent value in the range [0, 1).
     */
    private static double mod1(double value) {
        value %= 1.0;
        return value < 0 ? value + 1.0 : value;
    }

    /**
     * Computes the shortest distance between two rotational positions.
     *
     * <p>For example, the distance between 0.99 and 0.01 rotations is 0.02 rather than 0.98.
     *
     * @param a First position in rotations.
     * @param b Second position in rotations.
     * @return Circular distance in rotations.
     */
    private static double circularDistance(double a, double b) {
        double diff = Math.abs(a - b);
        return Math.min(diff, 1.0 - diff);
    }

    /**
     * Computes the greatest common divisor using Euclid's algorithm.
     *
     * @param a First value.
     * @param b Second value.
     * @return Greatest common divisor.
     */
    private static int gcd(int a, int b) {
        while (b != 0) {
            int t = a % b;
            a = b;
            b = t;
        }
        return Math.abs(a);
    }

    /**
     * Computes the least common multiple.
     *
     * @param a First value.
     * @param b Second value.
     * @return Least common multiple.
     */
    private static int lcm(int a, int b) {
        return a / gcd(a, b) * b;
    }
}