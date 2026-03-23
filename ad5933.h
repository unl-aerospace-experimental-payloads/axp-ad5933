#ifndef AD5933_H
#define AD5933_H

#include <Arduino.h>
#include <Wire.h>

/**
 * @brief Driver for the AD5933 impedance converter and network analyzer.
 *
 * The AD5933 generates a sine wave output, measures the response current,
 * and returns real/imaginary components of the transfer function via I2C.
 * Raw values must be calibrated against a known reference impedance to
 * obtain meaningful impedance measurements.
 *
 * Typical usage:
 *   1. Call initFrequencySweepParam() to configure and start a sweep
 *   2. Call Measure(true) repeatedly to step through frequencies
 *   3. Read `real` and `imaginary` after each Measure() call
 */
class AD5933
{
public:
    /**
     * @brief Construct an AD5933 driver instance.
     *
     * @param wire    Reference to the TwoWire (I2C) bus to use (e.g. Wire, Wire1, Wire2).
     *                On Teensy 4.1: Wire  = pins 18/19, Wire1 = pins 17/16, Wire2 = pins 25/24.
     * @param useExternalClock  Clock source select.
     *                true  = use an external clock signal on the MCLK pin.
     *                false = use the AD5933's internal 16.776 MHz oscillator (typical default).
     */
    AD5933(TwoWire& wire, bool useExternalClock);

    /**
     * @brief Trigger a single impedance measurement at the current or next frequency.
     *
     * Waits for the settling time configured in setSettlingTime(), then reads
     * the real and imaginary registers into the public `real` and `imaginary` members.
     *
     * @param incrementFrequency  Frequency step behavior:
     *                            true  = advance to the next frequency in the sweep
     *                                    (sends Increment Frequency command)
     *                            false = repeat measurement at the current frequency
     *                                    (sends Repeat Frequency command; useful for
     *                                     averaging or noise characterization)
     * @return true on success, false if the ADC did not signal valid data within
     *         the timeout or if any I2C transaction failed.
     */
    bool Measure(bool incrementFrequency);

    /**
     * @brief Reset the device, clearing sweep state while preserving register contents.
     *
     * After reset, the device enters standby. A new initFrequencySweepParam()
     * call is required before measuring again.
     *
     * @return true on success, false if any I2C transaction failed.
     */
    bool reset();

    /**
     * @brief Measure and return the internal die temperature.
     *
     * Sends the measure-temperature command and polls the status register
     * until the temperature conversion is complete.
     *
     * @return Temperature in degrees Celsius, or -1.0 if the conversion timed out.
     */
    float getTemperature();

    /**
     * @brief Place the device into standby mode.
     *
     * In standby, the output amplifier is disabled but I2C and internal
     * registers remain active. Required before issuing Initialize with
     * Start Frequency command.
     *
     * @return true on success, false if any I2C transaction failed.
     */
    bool standby();

    /**
     * @brief Place the device into power-down mode.
     *
     * Shuts down all internal circuitry except the I2C interface.
     * Lowest power consumption state. Call reset() or standby() to
     * return to an operational state.
     *
     * @return true on success, false if any I2C transaction failed.
     */
    bool powerdown();

    /**
     * @brief Configure all sweep parameters and perform the first measurement.
     *
     * Convenience method that calls setFrequencySweepParam(), setSettlingTime(),
     * standby(), setAnalogCircuit(), then sequences the Initialize/Start Frequency
     * Sweep control commands. After this returns, `real` and `imaginary` hold
     * the first data point. Call Measure(true) for each subsequent step.
     *
     * @param startFrequency   Start of the sweep in Hz (max ~100 kHz with internal clock).
     * @param stepFrequency    Frequency increment per step in Hz.
     * @param numberOfSteps    Total number of frequency increments (max 511).
     * @param numberOfCycles   Settling cycles before each ADC sample (see setSettlingTime()).
     * @param enablePGAGainX1  PGA gain (see setAnalogCircuit()).
     * @param voltageRange     Output voltage range 1–4 (see setAnalogCircuit()).
     * @return true on success, false if any I2C transaction or ADC read failed.
     */
    bool initFrequencySweepParam(unsigned int startFrequency, unsigned int stepFrequency,
                                 unsigned int numberOfSteps,  unsigned int numberOfCycles,
                                 bool enablePGAGainX1, int voltageRange);

    /// Raw real component of the last impedance measurement (uncalibrated ADC code).
    int real;

    /// Raw imaginary component of the last impedance measurement (uncalibrated ADC code).
    int imaginary;

private:
    TwoWire& _wire;
    uint8_t PGAandVoltout;  ///< Cached value of control register bits [2:0] for PGA and voltage range.
    bool _useExternalClock;

    /**
     * @brief Set the number of output cycles to produce before sampling begins.
     *
     * After the AD5933 switches to a new frequency, it waits this many cycles
     * of the output sine wave before triggering the ADC. More cycles allow the
     * system under test to reach steady state, which improves accuracy at the
     * cost of sweep speed.
     *
     * @param numberOfCycles  Settling cycle count, 1–2044.
     *                        Values above 1022 are internally divided to fit
     *                        the 10-bit register with a 4x multiplier.
     * @return true on success, false if any I2C transaction failed.
     */
    bool setSettlingTime(unsigned int numberOfCycles);

    /**
     * @brief Configure the output voltage range and programmable gain amplifier.
     *
     * Output voltage range controls the amplitude of the sine wave driven into
     * the impedance under test. Lower voltages reduce power dissipation in the
     * DUT and are safer for sensitive components.
     *
     * The PGA (Programmable Gain Amplifier) sits between the current-to-voltage
     * converter and the ADC input. Use x5 gain for high-impedance loads (weak
     * signal), and x1 gain for low-impedance loads to avoid ADC saturation.
     *
     * @param enablePGAGainX1  Gain setting for the receive-path amplifier.
     *                         true  = x1 gain (use for lower impedances, stronger signal)
     *                         false = x5 gain (use for higher impedances, weaker signal)
     * @param voltageRange     Output excitation voltage:
     *                         1 = 2.0 Vpp (maximum, best SNR for robust loads)
     *                         2 = 1.0 Vpp
     *                         3 = 0.4 Vpp
     *                         4 = 0.2 Vpp (minimum, safest for sensitive DUTs)
     * @return true on success, false if any I2C transaction failed.
     */
    bool setAnalogCircuit(bool enablePGAGainX1, int voltageRange);

    /// Write start frequency, frequency increment, and step count to their respective registers.
    bool setFrequencySweepParam(unsigned int startFrequency, unsigned int stepFrequency, unsigned int numberOfSteps);

    /// Issue the Address Pointer command (0xB0) to set the internal register read pointer.
    bool gotoAddressPointer(uint8_t registerAddress);

    /// Write a single byte to a register.
    bool setRegister(uint8_t registerAddress, uint8_t registerValue);

    /// Read a single byte from a register.
    uint8_t getRegister(uint8_t registerAddress);

    /**
     * @brief Check if a particular operation has completed or not.
     *
     * @param bitMask Bit mask to check in the status register.
     *      0x1 = valid temperature measurement
     *      0x2 = valid real/imaginary measurement   
     *      0x4 = frequency sweep complete
     * @return true if the specified bits are set, false otherwise.
     */
    bool getStatusBit(uint8_t bitMask);

    /// Read a contiguous block of bytes starting from the current address pointer.
    bool readBlock(uint8_t* byteArray, uint8_t numberOfBytes);

    /**
     * @brief Write to Control Register 1 (0x80).
     *
     * The AD5933 control register encodes both the command (bits [7:4]) and the
     * analog circuit config (PGA + voltage range, bits [2:0]). This method ORs
     * the given command nibble with the cached PGAandVoltout value so the analog
     * settings are never accidentally cleared.
     *
     * Valid command values (upper nibble):
     *   0x10 = Initialize with Start Frequency
     *   0x20 = Start Frequency Sweep
     *   0x30 = Increment Frequency
     *   0x40 = Repeat Frequency
     *   0x90 = Measure Temperature
     *   0xA0 = Power-Down
     *   0xB0 = Standby
     */
    bool setControlReg(uint8_t command);

    /// Poll the status register until ADC data is valid, then read real/imaginary registers.
    bool getData();
};

#endif