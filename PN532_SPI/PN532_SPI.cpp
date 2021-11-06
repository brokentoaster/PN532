
#include "PN532/PN532_SPI/PN532_SPI.h"
#include "PN532/PN532/PN532_debug.h"
#include "Arduino.h"

#define STATUS_READ 2
#define DATA_WRITE 1
#define DATA_READ 3

#define PN532_SPI_WAKE_DELAY_MS  2
#define PN532_SPI_SELECT_DELAY_MS 1

PN532_SPI::PN532_SPI(SPIClass &spi, uint8_t ss)
{
    command = 0;
    _spi = &spi;
    _ss = ss;
}

void PN532_SPI::begin()
{
    pinMode(_ss, OUTPUT);

    _spi->begin();
    _spi->setDataMode(SPI_MODE0); // PN532 only supports mode0
    _spi->setBitOrder(LSBFIRST);
#if defined __SAM3X8E__
    /** DUE spi library does not support SPI_CLOCK_DIV8 macro */
    _spi->setClockDivider(42); // set clock 2MHz(max: 5MHz)
#elif defined __SAMD21G18A__
    /** M0 spi library does not support SPI_CLOCK_DIV8 macro */
    _spi->setClockDivider(24); // set clock 2MHz(max: 5MHz)
#else
    _spi->setClockDivider(SPI_CLOCK_DIV8); // set clock 2MHz(max: 5MHz)
#endif
}

void PN532_SPI::wakeup()
{
    digitalWrite(_ss, LOW);
    delay(PN532_SPI_WAKE_DELAY_MS);
    digitalWrite(_ss, HIGH);
}

int8_t PN532_SPI::writeCommand(const uint8_t *header, uint8_t hlen, const uint8_t *body, uint8_t blen)
{
    command = header[0];
    writeFrame(header, hlen, body, blen);

    const uint8_t waitReady = isReady( PN532_ACK_WAIT_TIME);
    if(waitReady != PN532_SUCCESS)
    {
        DMSG("Time out when waiting for ACK\n");
        return PN532_TIMEOUT;
    }
    
    if (readAckFrame())
    {
        DMSG("Invalid ACK\n");
        return PN532_INVALID_ACK;
    }
    return PN532_SUCCESS;
}

int16_t PN532_SPI::readResponse(uint8_t buf[], uint8_t buflen, uint16_t timeout)
{
    const uint8_t waitReady = isReady( timeout);
    if(waitReady != PN532_SUCCESS)
    {
        return PN532_TIMEOUT;
    }

    digitalWrite(_ss, LOW);
    delay(PN532_SPI_SELECT_DELAY_MS);

    int16_t result = readFrame(buff,buflen);

    digitalWrite(_ss, HIGH);

    return result;
}


bool readFramePreambleOk( void )
{
    
    if (0x00 != read() || // PREAMBLE
        0x00 != read() || // STARTCODE1
        0xFF != read()    // STARTCODE2
    )
    {
        return  false;
    }
    
    return true;

}

int16_t readFrameLength( void )
{
    const uint8_t length = read();
    const uint8_t length_cs = read();
    const uint8_t result = length + length_cs ;

    if (result != 0)
    { // checksum of length
        return PN532_INVALID_FRAME;
    }

    return length;
}

int16_t PN532::readFrame((uint8_t buf[], uint8_t buflen )
{
        write(DATA_READ);

  
        if ( !readFramePreambleOk() )
        {
            return PN532_INVALID_FRAME;
        }


     if (result != 0)
    { // checksum of length
        return PN532_INVALID_FRAME;
    }



        uint8_t cmd = command + 1; // response command
        if (PN532_PN532TOHOST != read() || (cmd) != read())
        {
            result = PN532_INVALID_FRAME;
            return result;
        }



        DMSG("read:  ");
        DMSG_HEX(cmd);

        length -= 2;
        if (length > buflen)
        {
            for (uint8_t i = 0; i < length; i++)
            {
                DMSG_HEX(read()); // dump message
            }
            DMSG("\nNot enough space\n");
            read();
            read();
            result = PN532_NO_SPACE; // not enough space
            return result;
        }

        uint8_t sum = PN532_PN532TOHOST + cmd;
        for (uint8_t i = 0; i < length; i++)
        {
            buf[i] = read();
            sum += buf[i];

            DMSG_HEX(buf[i]);
        }
        DMSG('\n');



        uint8_t checksum = read();
        if (0 != (uint8_t)(sum + checksum))
        {
            DMSG("checksum is not ok\n");
            result = PN532_INVALID_FRAME;
            return result;
        }
        read(); // POSTAMBLE

        result = length;
    
    return result;
}

bool PN532_SPI::isReady()
{
    digitalWrite(_ss, LOW);

    write(STATUS_READ);
    uint8_t status = read() & 1;
    digitalWrite(_ss, HIGH);
    return status;
}

int8_t PN532_SPI::isReady( const uint16_t timeout_ms )
{
    uint16_t time = 0;
    while (!isReady())
    {
        delay(1);
        time++;
        if (time > timeout_ms)
        {
            return PN532_TIMEOUT;
        }
    }
    return PN532_SUCCESS;
}

void PN532_SPI::writeFrame(const uint8_t *header, uint8_t hlen, const uint8_t *body, uint8_t blen)
{

    digitalWrite(_ss, LOW);
    delay(PN532_SPI_WAKE_DELAY_MS); 

    const uint8_t dataLength = hlen + blen + 1; // length of data field: TFI + DATA
    writeFrameHeader(dataLength);

    writeFrameData(header,hlen,body,blen);



    digitalWrite(_ss, HIGH);

    DMSG('\n');
}



void PN532_SPI::writeFrameHeader(const uint8_t length)
{
    write(DATA_WRITE);
    write(PN532_PREAMBLE);
    write(PN532_STARTCODE1);
    write(PN532_STARTCODE2);

    write(length);      // LEN
    write(~length + 1); // LCS checksum of length
}



//
// Write the data portion of the frame.
//
// Notes from the user manual..
//
// TFI: 1 byte frame identifier, the value of this byte depends
// on the way of the message
// - D4h in case of a frame from the host controller to the PN532,
// - D5h in case of a frame from the PN532 to the host controller
//
// DATA: LEN-1 bytes of Packet Data Information
// The first byte PD0 is the Command Code.
//
// DCS: 1 Data Checksum DCS byte that satisfies the relation:
// Lower byte of [TFI + PD0 + PD1 + … + PDn + DCS] = 0x00,
//
// Note:
// Dump the block in one go ate the end so it doesn't interfer 
// with the timming of writes to the bus when debugging vs when 
// not debugging
void PN532_SPI::writeFrameData(const uint8_t *header, uint8_t hlen, const uint8_t *body, uint8_t blen)
{
    uint8_t sum = 0; 

    // TFI
    write(PN532_HOSTTOPN532); 
    sum += PN532_HOSTTOPN532; 
    
    // DATA
    sum += writeBlockandSum( header, hlen);
    sum += writeBlockandSum( body, blen);

    // DCS
    write(~sum+1); 
    
    write(PN532_POSTAMBLE);

    // Debugging info
    DMSG("write: ");
    dmsgBlock( header, hlen );
    dmsgBlock( body,   blen );
}



uint8_t PN532_SPI::writeBlockandSum(uint8_t const * const data, const uint8_t length)
{
    uint8_t sum = 0;

    for (uint8_t i = 0; i < length; i++)
    {
        const uint8_t d = data[i];
        write(d);
        sum += d;
    }
    return sum;
}


void PN532_SPI::dmsgBlock(uint8_t const * const data, const uint8_t length)
{
    for (uint8_t i = 0; i < length; i++)
    {
        const uint8_t d = data[i];
        DMSG_HEX(d);
    }
    
}

int8_t PN532_SPI::readAckFrame()
{
    const uint8_t PN532_ACK[] = {0, 0, 0xFF, 0, 0xFF, 0};

    uint8_t ackBuf[sizeof(PN532_ACK)];

    digitalWrite(_ss, LOW);
    delay(1);
    write(DATA_READ);

    for (uint8_t i = 0; i < sizeof(PN532_ACK); i++)
    {
        ackBuf[i] = read();
    }

    digitalWrite(_ss, HIGH);

    return memcmp(ackBuf, PN532_ACK, sizeof(PN532_ACK));
}
