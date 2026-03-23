#ifdef mbed
#include "ad5933.h"
#include "mbed.h"

// Define Command bytes
#define INIT_FREQ  0x10     // initialise startfreq
#define INIT_SWEEP 0x20     // initialise sweep
#define INCR_FREQ  0x30     // increment frequency
#define REPE_FREQ  0x40     // repeat frequency
#define STANDBY    0xB0     // standby
#define POWERDOWN  0xA0     // PowerDown modus
#define MEAS_TEMP  0x90     // temperature

#define WRITE_CMD  0x1A     // adress + write command
#define READ_CMD   0x1B     // adress + read command

#define CLOCK_FREQ 0x00F42400
#define I2C_FREQ   400000

#define WAITTIME   1800     // time to wait before polling for response

AD5933::AD5933(PinName sda, PinName scl, bool extClk) : sCom(sda, scl)
{
    sCom.frequency(I2C_FREQ);
    PGAandVoltout = 0x00;
    _extClk = extClk;
}

bool AD5933::gotoAdressPointer(uint8_t Adress)
{
    sCom.start();
    bool output = (sCom.write(WRITE_CMD) + sCom.write(0xB0) + sCom.write(Adress)) == 3;
    sCom.stop();
    return output;
}

bool AD5933::setRegister(uint8_t RegisterAdress, uint8_t RegisterValue)
{
    sCom.start();
    bool output = (sCom.write(WRITE_CMD) + sCom.write(RegisterAdress) + sCom.write(RegisterValue)) == 3;
    sCom.stop();
    return output;
}

bool AD5933::writeBlock(uint8_t ByteArray[], uint8_t sizeArray)
{
    sCom.start();
    bool output = (sCom.write(WRITE_CMD) + sCom.write(0xA0) + sCom.write(sizeArray)) == 3;
    for(uint8_t i = 0; i<sizeArray; i++) {
        output = sCom.write(ByteArray[i]) == 1 && output;
    }
    sCom.stop();
    return output;
}

uint8_t AD5933::getRegister(uint8_t RegisterAdress)
{
    gotoAdressPointer(RegisterAdress);

    uint8_t output = 0xFF;
    sCom.start();
    if(sCom.write(READ_CMD) == 1)
        output = sCom.read(0);
    sCom.stop();
    return output;
}

bool AD5933::readBlock(uint8_t* ByteArray, uint8_t sizeArray)
{
    sCom.start();
    bool output = (sCom.write(WRITE_CMD) + sCom.write(0xA1) + sCom.write(sizeArray)) == 3;
    sCom.start();
    output = output && (sCom.write(READ_CMD) == 1);
    for(uint8_t i = 0; i<sizeArray-1; i++) {
        ByteArray[i] = sCom.read(1);
    }
    ByteArray[sizeArray-1] = sCom.read(0);
    sCom.stop();
    return output;
}

bool AD5933::setControlReg(uint8_t Command)
{
    return setRegister(0x80, PGAandVoltout | Command);
}

bool AD5933::setFrequencySweepParam(unsigned int startFreq, unsigned int stepFreq, unsigned int nrOfSteps)
{
    unsigned int startFreqCode = startFreq/CLOCK_FREQ*0x00000004*0x08000000;
    unsigned int stepFreqCode = stepFreq/CLOCK_FREQ*0x00000004*0x08000000;

    bool output = setRegister(0x82,(startFreqCode >> 16));
    output &= setRegister(0x83,(startFreqCode >> 8));
    output &= setRegister(0x84,(startFreqCode));
    output &= setRegister(0x85,(stepFreqCode >> 16));
    output &= setRegister(0x86,(stepFreqCode >> 8));
    output &= setRegister(0x87,(stepFreqCode));
    output &= setRegister(0x88,(nrOfSteps >> 8));
    output &= setRegister(0x89,nrOfSteps);

    return output;
}

bool AD5933::initFrequencySweepParam(unsigned int startFreq, unsigned int stepFreq, unsigned int nrOfSteps, unsigned int nrOfCycles, bool PGA, int RangeNr)
{
    bool output = setFrequencySweepParam(startFreq, stepFreq, nrOfSteps);
    output &= setSettlingTime(nrOfCycles);
    output &= standby();
    output &= setAnalogCircuit(PGA, RangeNr);
    output &= setControlReg(INIT_FREQ);
    wait_ms(5);
    output &= setControlReg(INIT_SWEEP);
    wait_us(WAITTIME);
    output &= getData();

    return output;
}

bool AD5933::setSettlingTime(unsigned int nrOfCycles)
{
    bool output = true;

    if (nrOfCycles > 1022) {
        output &= setRegister(0x8A,((nrOfCycles/4) >> 8) | 0x06);
        output &= setRegister(0x8B,(nrOfCycles/4));
    } else if(nrOfCycles > 511) {
        output &= setRegister(0x8A,((nrOfCycles/4) >> 8) | 0x02);
        output &= setRegister(0x8B,(nrOfCycles/2));
    } else {
        output &= setRegister(0x8A,0x00);
        output &= setRegister(0x8B,nrOfCycles);
    }
    return output;
}

bool AD5933::setAnalogCircuit(bool PGA, int RangeNr)
{
    if(PGA)
        PGAandVoltout = 0x01;
    else
        PGAandVoltout = 0x00;

    switch(RangeNr) {
        case 1:
            PGAandVoltout |= 0x00;
        case 2:
            PGAandVoltout |= 0x06;
            break;
        case 3:
            PGAandVoltout |= 0x04;
            break;
        case 4:
            PGAandVoltout |= 0x02;
            break;
    }

    uint8_t data = 0x00;
    if(_extClk)
        data |= 0x08;

    bool output = setRegister(0x81, data);
    output &= setRegister(0x80,PGAandVoltout);

    return output;
}

bool AD5933::reset()
{
    uint8_t data = 0x10;
    if(_extClk)
        data |= 0x08;

    return setRegister(0x81, data);
}

bool AD5933::standby()
{
    return setControlReg(STANDBY);
}

bool AD5933::powerdown()
{
    return setControlReg(POWERDOWN);
}

bool AD5933::Measure(bool increment)
{
    if(increment) {
        setControlReg(INCR_FREQ);
        wait_us(WAITTIME);
        return getData();
    } else {
        setControlReg(0x00);
        setControlReg(REPE_FREQ);
        wait_us(WAITTIME);
        return getData();
    }
}

bool AD5933::getData()
{
    int i = 0;
    uint8_t data[4];
    bool output;

    while(((getRegister(0x8F) & 0x02)  != 0x02) && i < 10) {
        wait_us(500);
        i++;
    }
    if(i == 10)
        output = false;

    output &= gotoAdressPointer(0x82);
    output &= readBlock(data, 4);
    real = data[0] << 8 | data[1];
    imaginary = data[2] << 8 | data[3];
    return output;
}

float AD5933::getTemperature()
{
    int i = 0;
    uint8_t data[2];

    setControlReg(MEAS_TEMP);
    wait_us(WAITTIME);

    while(((getRegister(0x8F) & 0x01) != 0x01) && i < 10) {
        wait_us(500);
        i++;
    }
    if(i == 10)
        return -1;

    gotoAdressPointer(0x92);
    readBlock(data, 2);

    if((data[0] >> 6) & 1) {
        //negative temperature
        return (((data[0] << 8 | data[1]) - 16384)/32.0) ;
    } else {
        return ((data[0] << 8 | data[1])/32.0) ;
    }
}
#endif