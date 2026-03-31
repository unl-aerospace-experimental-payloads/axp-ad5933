#include "ad5933.h"

#define COMMAND_INITIALIZE_WITH_START_FREQUENCY  0x10
#define COMMAND_START_FREQUENCY_SWEEP            0x20
#define COMMAND_INCREMENT_FREQUENCY              0x30
#define COMMAND_REPEAT_FREQUENCY                 0x40
#define COMMAND_STANDBY                          0xB0
#define COMMAND_POWER_DOWN                       0xA0
#define COMMAND_MEASURE_TEMPERATURE              0x90

#define AD5933_I2C_ADDRESS                       0x0D
#define AD5933_INTERNAL_CLOCK_FREQUENCY_HZ       16776000UL  // 16.776 MHz internal oscillator
#define ADC_SETTLING_WAIT_MICROSECONDS           1800        // microseconds to wait after issuing a frequency command before polling for valid ADC data
#define POLLING_TIMEOUT_COUNT                    50
#define POLLING_WAIT_MICROSECONDS                1000         // microseconds to wait between polling attempts for valid ADC data or temperature data

#define STATUS_REGISTER                          0x8F
#define TEMPERATURE_VALID_BIT                    0x01
#define REAL_IMAGINARY_VALID_BIT                 0x02
#define FREQUENCY_SWEEP_COMPLETE_BIT             0x04


AD5933::AD5933(TwoWire& wire, bool useExternalClock) : _wire(wire), _useExternalClock(useExternalClock)
{
    PGAandVoltout = 0x00;
    _wire.begin();
    _wire.setClock(400000);
    _startFrequency = 0;
    _stepFrequency = 0;
    _numberOfSteps = 0;
    _currentStep = 0;
}

bool AD5933::gotoAddressPointer(uint8_t address)
{
    _wire.beginTransmission(AD5933_I2C_ADDRESS);
    _wire.write(0xB0);  // address pointer command
    _wire.write(address);
    return _wire.endTransmission() == 0;
}

bool AD5933::setRegister(uint8_t registerAddress, uint8_t registerValue)
{
    _wire.beginTransmission(AD5933_I2C_ADDRESS);
    _wire.write(registerAddress);
    _wire.write(registerValue);
    return _wire.endTransmission() == 0;
}

uint8_t AD5933::getRegister(uint8_t registerAddress)
{
    gotoAddressPointer(registerAddress);

    _wire.requestFrom((uint8_t)AD5933_I2C_ADDRESS, (uint8_t)1);
    if (_wire.available())
        return _wire.read();
    return 0xFF;
}

bool AD5933::getStatusBit(uint8_t bitMask)
{
    return (getRegister(STATUS_REGISTER) & bitMask) == bitMask;
}

bool AD5933::readBlock(uint8_t* byteArray, uint8_t size)
{
    // Use block read command
    _wire.beginTransmission(AD5933_I2C_ADDRESS);
    _wire.write(0xA1);  // block read command
    _wire.write(size);
    bool ok = (_wire.endTransmission() == 0);

    _wire.requestFrom((uint8_t)AD5933_I2C_ADDRESS, size);
    for (uint8_t i = 0; i < size; i++) {
        if (_wire.available())
            byteArray[i] = _wire.read();
        else {
            ok = false;
            byteArray[i] = 0xFF;
        }
    }
    return ok;
}

bool AD5933::setControlReg(uint8_t command)
{
    return setRegister(0x80, PGAandVoltout | command);
}

bool AD5933::setFrequencySweepParam(unsigned int startFrequency, unsigned int stepFrequency, unsigned int numberOfSteps)
{
    // Frequency code = (freq / (CLOCK_FREQ / 4)) * 2^27
    uint32_t startCode = (uint32_t)(((uint64_t)startFrequency << 29) / (AD5933_INTERNAL_CLOCK_FREQUENCY_HZ / 2));
    uint32_t stepCode  = (uint32_t)(((uint64_t)stepFrequency  << 29) / (AD5933_INTERNAL_CLOCK_FREQUENCY_HZ / 2));

    bool ok = true;
    // Write to the Start Frequency registers (0x82–0x84)
    ok &= setRegister(0x82, (startCode >> 16) & 0xFF);
    ok &= setRegister(0x83, (startCode >>  8) & 0xFF);
    ok &= setRegister(0x84,  startCode        & 0xFF);
    // Write to the Frequency Increment registers (0x85–0x87)
    ok &= setRegister(0x85, (stepCode  >> 16) & 0xFF);
    ok &= setRegister(0x86, (stepCode  >>  8) & 0xFF);
    ok &= setRegister(0x87,  stepCode         & 0xFF);
    // Write to the Number of Increments registers (0x88–0x89)
    ok &= setRegister(0x88, (numberOfSteps >>  8) & 0xFF);
    ok &= setRegister(0x89,  numberOfSteps        & 0xFF);
    return ok;
}

bool AD5933::initFrequencySweepParam(uint32_t startFrequency, uint32_t stepFrequency, uint16_t numberOfSteps, unsigned int settlingCycles, bool enablePGAGainX1, int voltageRange)
{
    bool ok = setFrequencySweepParam(startFrequency, stepFrequency, numberOfSteps);

    if (ok) {
        _startFrequency = startFrequency;
        _stepFrequency = stepFrequency;
        _numberOfSteps = numberOfSteps;
        _currentStep = 0;
    }

    ok &= setSettlingTime(settlingCycles);
    ok &= standby();
    ok &= setAnalogCircuit(enablePGAGainX1, voltageRange);
    ok &= setControlReg(COMMAND_INITIALIZE_WITH_START_FREQUENCY);
    delay(5);
    ok &= setControlReg(COMMAND_START_FREQUENCY_SWEEP);
    delayMicroseconds(ADC_SETTLING_WAIT_MICROSECONDS);
    ok &= getData();
    return ok;
}

bool AD5933::setSettlingTime(unsigned int numberOfCycles)
{
    bool ok = true;
    if (numberOfCycles > 1022) {
        ok &= setRegister(0x8A, ((numberOfCycles / 4) >> 8) | 0x06);
        ok &= setRegister(0x8B, (numberOfCycles / 4) & 0xFF);
    } else if (numberOfCycles > 511) {
        ok &= setRegister(0x8A, ((numberOfCycles / 2) >> 8) | 0x02);
        ok &= setRegister(0x8B, (numberOfCycles / 2) & 0xFF);
    } else {
        ok &= setRegister(0x8A, 0x00);
        ok &= setRegister(0x8B, numberOfCycles & 0xFF);
    }
    return ok;
}

bool AD5933::setAnalogCircuit(bool PGA, int RangeNr)
{
    // From Table 11. Control Register Map (D11, D8 to D0)
    // D8 PGA gain; 0 = ×5, 1 = ×1 
    // D10 to D9 is for voltage range. (Table 10)

    PGAandVoltout = PGA ? 0x01 : 0x00;

    switch (RangeNr) {
        case 1: PGAandVoltout |= 0x00; break;
        case 2: PGAandVoltout |= 0x06; break;
        case 3: PGAandVoltout |= 0x04; break;
        case 4: PGAandVoltout |= 0x02; break;
    }

    uint8_t ctrl2 = _useExternalClock ? 0x08 : 0x00;
    bool ok = setRegister(0x81, ctrl2);
    //PGAandVoltout is saved as a global variable and applied when we set the control register.
    // ok &= setRegister(0x80, PGAandVoltout);
    return ok;
}

bool AD5933::reset()
{
    uint8_t data = 0x10;
    if (_useExternalClock) data |= 0x08;
    return setRegister(0x81, data);
}

bool AD5933::standby()
{
    return setControlReg(COMMAND_STANDBY);
}

bool AD5933::powerdown()
{
    return setControlReg(COMMAND_POWER_DOWN);
}

uint32_t AD5933::getFrequency()
{
    return _startFrequency + (_currentStep * _stepFrequency);
}

bool AD5933::Measure(bool increment)
{
    if (increment) {
        setControlReg(COMMAND_INCREMENT_FREQUENCY);
        _currentStep++; // THis could potentially before the device actually increments the frequency, but we will assume it works for now. If we wanted to be more robust, we could check the frequency sweep complete bit and reset currentStep back to 0 if we see it set.
    } else {
        setControlReg(0x00);
        setControlReg(COMMAND_REPEAT_FREQUENCY);
    }
    if (getStatusBit(FREQUENCY_SWEEP_COMPLETE_BIT)) {
        if (_currentStep != _numberOfSteps) {
            Serial.println("Warning: AD5933 frequency sweep complete bit is set, but currentStep does not equal numberOfSteps. This may indicate that the driver and device are out of sync on the current step count.");
        }
        // Sweep complete. Reset currentStep back to 0 so that getFrequency() returns to startFrequency until the next sweep starts.
        _currentStep = 0;
    }
    delayMicroseconds(ADC_SETTLING_WAIT_MICROSECONDS);
    bool ok = getData(); 
    return ok;
}

bool AD5933::getData()
{
    int i = 0;
    while (!getStatusBit(REAL_IMAGINARY_VALID_BIT) && i < POLLING_TIMEOUT_COUNT) {
        delayMicroseconds(POLLING_WAIT_MICROSECONDS);
        i++;
    }

    if (i == POLLING_TIMEOUT_COUNT) {
        Serial.println("AD5933: ADC data not valid after polling. Timed out.");
        return false;
    }

    uint8_t r_hi = getRegister(0x94);
    uint8_t r_lo = getRegister(0x95);
    uint8_t i_hi = getRegister(0x96);
    uint8_t i_lo = getRegister(0x97);

    real      = (int16_t)((uint16_t)r_hi << 8 | r_lo);
    imaginary = (int16_t)((uint16_t)i_hi << 8 | i_lo);

    return true;
}

float AD5933::getTemperature()
{
    setControlReg(COMMAND_MEASURE_TEMPERATURE);
    delayMicroseconds(ADC_SETTLING_WAIT_MICROSECONDS);

    int i = 0;
    while (!getStatusBit(TEMPERATURE_VALID_BIT) && i < POLLING_TIMEOUT_COUNT) {
        delayMicroseconds(POLLING_WAIT_MICROSECONDS);
        i++;
    }
    if (i == POLLING_TIMEOUT_COUNT) return -1.0f;

    uint8_t data[2];
    gotoAddressPointer(0x92);
    readBlock(data, 2);

    int16_t raw = (data[0] << 8) | data[1];
    if (raw & 0x2000) {
        // negative: 14-bit two's complement
        raw |= 0xC000;
    }
    return raw / 32.0f;
}