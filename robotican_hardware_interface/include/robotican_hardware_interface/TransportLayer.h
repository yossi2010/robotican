//
// Created by tom on 05/05/16.
//

#ifndef ROBOTICAN_HARDWARE_INTERFACE_TRANSPORTLAYER_H
#define ROBOTICAN_HARDWARE_INTERFACE_TRANSPORTLAYER_H

#include <robotican_hardware_interface/Protocol.h>
#include <boost/asio/serial_port.hpp>
#include <boost/asio.hpp>

typedef uint16_t crc;

#define POLYNOMIAL 0xD8  /* 11011 followed by 0's */
#define WIDTH  (8 * sizeof(crc))
#define TOPBIT (1 << (WIDTH - 1))
#define MAX_TABLE 256

#include <robotican_hardware_interface/ros_utils.h>

class TransportLayer {
private:
    crc _crcTable[MAX_TABLE];
    boost::asio::io_service _ioService;
    boost::asio::serial_port _serial;                                                                                   //!< The serial channel
    /*!
     * @brief Method will read packet from the RiCBoard and store it in a buffer
     * @param buff: The store buffer
     * @param buffLength: The max buffer length
     * @return If succeed to read pkg or not
     */
    bool read(byte buff[], byte buffLength);
    /*!
     * @brief This method will call ones at init, all this method does is init the '_crcTable' field.
     */
    void crcInit();

public:
    TransportLayer(std::string port, unsigned int baudrate);
    ~TransportLayer();
    /*!
     * @brief Method that try to read a valid RiCBoard pkg.
     * @param buff: The buffer that store the valid pkg
     * @param buffLength: The max buffer length
     * @return If succeed to read pkg or not
     */
    bool tryToRead(byte *buff, byte buffLength);
    /*!
     * @brief Method that write pkg to the RiCBoard.
     * @param buff: Buffer that store the pkg to be send
     * @param buffLength: How many byte needed to send to the RiCBoard
     */
    void write(byte buff[], byte buffLength);
    /*!
     * @brief Calculate the crc checksum
     * @param message: msg to be calculated
     * @param nBytes: the bytes length
     * @return The checksum result
     */
    crc calcChecksum(uint8_t const message[], int nBytes);
};

#endif //ROBOTICAN_HARDWARE_INTERFACE_TRANSPORTLAYER_H
