#ifndef INRIA_UTILS_TRANSPORTVALUEUDP_HPP
#define INRIA_UTILS_TRANSPORTVALUEUDP_HPP

#include <string>
#include <vector>
#include <stdexcept>
#include <atomic>
#include <chrono>
#include <thread>
#include <functional>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <poll.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>

/**
 * Implement Server and Client to transport named typed boolean, integer 
 * and floating point values across UDP network.
 * Design to be conveniently used with RhIO.
 */

namespace inria {

/**
 * Maximum length for full variable name
 */
constexpr unsigned int NAME_MAX_LENGTH = 80;

/**
 * TransportValueUDPServer
 *
 * Receiver and update processing values
 * from UDP packet
 */
class TransportValueUDPServer
{
    public:

        /**
         * Typedef for callback processing function
         */
        using Callback_t = std::function<void(
            const std::vector<std::string>& namesBool,
            const std::vector<std::string>& namesInt,
            const std::vector<std::string>& namesFloat,
            const std::vector<bool>& valuesBool,
            const std::vector<int64_t>& valuesInt,
            const std::vector<double>& valuesFloat)>;

        /**
         * Default initialization
         */
        TransportValueUDPServer() :
            _isContinue(false),
            _port(-1),
            _callback(),
            _thread(),
            _lastTime(0)
        {
        }

        /**
         * Stop listening thread
         */
        ~TransportValueUDPServer()
        {
            if (_isContinue.load()) {
                _isContinue.store(false);
                _thread.join();
            }
        }

        /**
         * Start listening thread with given
         * port and processing callback.
         *
         * @param port UDP port to listen to.
         * @param callback Received data processing callback.
         */
        void listen(unsigned int port, Callback_t callback)
        {
            if (!_isContinue.load()) {
                _isContinue.store(true);
                _port = port;
                _callback = callback;
                _thread = std::thread([this](){this->mainThread();});
            }
        }

    private:

        /**
         * True if the listening thread 
         * must continue running
         */
        std::atomic<bool> _isContinue;

        /**
         * Listening port
         */
        unsigned int _port;

        /**
         * Processing callback function
         */
        Callback_t _callback;

        /**
         * Listening thread instance
         */
        std::thread _thread;

        /**
         * Last received packet time in microseconds
         */
        uint32_t _lastTime;
        
        /**
         * Main function of receiving thread
         */
        void mainThread()
        {
            //Create IPv4 UDP socket
            unsigned int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
            if (sockfd == -1) {
                throw std::logic_error(
                    "inria::TransportValueUDPServer::mainThread: "
                    "Error creating socket: " +
                    std::string(strerror(errno)));
            }
            
            //Bind socket to listening port
            struct sockaddr_in addrServer;
            bzero(&addrServer, sizeof(addrServer));
            addrServer.sin_family = AF_INET;
            addrServer.sin_port = htons(_port);
            addrServer.sin_addr.s_addr = htonl(INADDR_ANY);
            int ret = bind(sockfd, 
                (const struct sockaddr *)&addrServer, sizeof(addrServer));
            if (ret == -1) {
                throw std::logic_error(
                    "inria::TransportValueUDPServer::mainThread: "
                    "Error binding socket: " +
                    std::string(strerror(errno)));
            }
            
            //Polling structure setup
            struct pollfd fds;
            fds.fd = sockfd;
            fds.events = POLLIN | POLLPRI;

            //Allocate packet buffer
            size_t maxBufferSize = NAME_MAX_LENGTH*1000;
            unsigned char* buffer = new unsigned char[maxBufferSize];
            //Create receiving container
            std::vector<std::string> namesBool;
            std::vector<std::string> namesInt;
            std::vector<std::string> namesFloat;
            std::vector<bool> valuesBool;
            std::vector<int64_t> valuesInt;
            std::vector<double> valuesFloat;

            //Main receiving loop
            while (_isContinue.load()) {
                //UDP reception
                //Check if data are available to be read with a timeout
                int retevent = poll(&fds, 1, 500);
                if (retevent == -1) {
                    throw std::logic_error(
                        "inria::TransportValueUDPServer::mainThread: "
                        "Error polling data: " +
                        std::string(strerror(errno)));
                }
                if (retevent == 0) {
                    continue;
                }
                int ret = recvfrom(
                    sockfd, buffer, maxBufferSize, 0, NULL, NULL);
                if (ret == -1) {
                    throw std::logic_error(
                        "inria::TransportValueUDPServer::mainThread: "
                        "Error receiving data: " +
                        std::string(strerror(errno)));
                }
                //Check packet validity
                if (ret < (sizeof(double)+3*sizeof(uint32_t)) || ret > maxBufferSize) {
                    //The message is invalid
                    std::cout << "Network: invalid packet received: " << ret << std::endl;
                    continue;
                }
                //Parse packet data
                namesBool.clear();
                namesInt.clear();
                namesFloat.clear();
                valuesBool.clear();
                valuesInt.clear();
                valuesFloat.clear();
                size_t index = 0;
                double lastTime = *reinterpret_cast<double*>(&buffer[index]);
                index += sizeof(double);
                uint32_t sizeBool = *reinterpret_cast<uint32_t*>(&buffer[index]);
                index += sizeof(uint32_t);
                uint32_t sizeInt = *reinterpret_cast<uint32_t*>(&buffer[index]);
                index += sizeof(uint32_t);
                uint32_t sizeFloat = *reinterpret_cast<uint32_t*>(&buffer[index]);
                index += sizeof(uint32_t);
                //Drop packets out of order
                if (lastTime <= _lastTime) {
                    continue;
                }
                _lastTime = lastTime;
                //Check packet size structure validity
                size_t sizePacket = 
                    sizeof(double) +
                    3*sizeof(uint32_t) +
                    sizeBool*(NAME_MAX_LENGTH + sizeof(uint8_t)) +
                    sizeInt*(NAME_MAX_LENGTH + sizeof(int64_t)) +
                    sizeFloat*(NAME_MAX_LENGTH + sizeof(double));
                if (ret != sizePacket) {
                    //The message is invalid
                    std::cout << "Network: invalid packet received: " << ret << std::endl;
                    continue;
                }
                //Extract data names and values
                char name[NAME_MAX_LENGTH];
                for (size_t i=0;i<sizeBool;i++) {
                    strncpy(name, reinterpret_cast<const char*>(&buffer[index]), NAME_MAX_LENGTH);
                    namesBool.push_back(std::string(name));
                    index += NAME_MAX_LENGTH;
                }
                for (size_t i=0;i<sizeInt;i++) {
                    strncpy(name, reinterpret_cast<const char*>(&buffer[index]), NAME_MAX_LENGTH);
                    namesInt.push_back(std::string(name));
                    index += NAME_MAX_LENGTH;
                }
                for (size_t i=0;i<sizeFloat;i++) {
                    strncpy(name, reinterpret_cast<const char*>(&buffer[index]), NAME_MAX_LENGTH);
                    namesFloat.push_back(std::string(name));
                    index += NAME_MAX_LENGTH;
                }
                for (size_t i=0;i<sizeBool;i++) {
                    valuesBool.push_back(*reinterpret_cast<uint8_t*>(&buffer[index]) > 0);
                    index += sizeof(uint8_t);
                }
                for (size_t i=0;i<sizeInt;i++) {
                    valuesInt.push_back(*reinterpret_cast<int64_t*>(&buffer[index]));
                    index += sizeof(int64_t);
                }
                for (size_t i=0;i<sizeFloat;i++) {
                    valuesFloat.push_back(*reinterpret_cast<double*>(&buffer[index]));
                    index += sizeof(double);
                }
                //Apply processing callback
                _callback(
                    namesBool, namesInt, namesFloat,
                    valuesBool, valuesInt, valuesFloat);
            }

            //Deallocate packet buffer
            delete[] buffer;

            //Close socket
            close(sockfd);
        }
};

/**
 * TransportValueUDPClient
 *
 * Send and set value to remote 
 * server through UDP packet
 */
class TransportValueUDPClient
{
    public:

        /**
         * Default initialization
         */
        TransportValueUDPClient() :
            _namesBool(),
            _namesInt(),
            _namesFloat(),
            _valuesBool(),
            _valuesInt(),
            _valuesFloat(),
            _sockfd(-1)
        {
        }

        /**
         * Close connection
         */
        ~TransportValueUDPClient()
        {
            if (_sockfd != -1) {
                close(_sockfd);
                _sockfd = -1;
            }
        }

        /**
         * Initialize client with given 
         * IP address to server
         *
         * @param ip IP address to server in textual format.
         * @param port UDP remote port.
         */
        void connect(const std::string& ip, unsigned int port)
        {
            //Create IPv4 UDP socket
            _sockfd = socket(AF_INET, SOCK_DGRAM, 0);
            if (_sockfd == -1) {
                throw std::logic_error(
                    "inria::TransportValueUDPClient::connect: "
                    "Error creating socket: " +
                    std::string(strerror(errno)));
            }

            //Convert IP address from hostname to numeric format
            std::string ipConverted = ip;
            struct addrinfo hints;
            struct addrinfo* result;
            memset(&hints, 0, sizeof(hints));
            hints.ai_family = AF_UNSPEC;
            hints.ai_socktype = SOCK_STREAM;
            hints.ai_flags |= AI_CANONNAME;
            int status = getaddrinfo(ip.c_str(), NULL, &hints, &result);
            if (status != 0) {
                throw std::logic_error(
                    "inria::TransportValueUDPClient::connect: "
                    "Error converting ip address: " +
                    std::string(strerror(errno)));
            }
            for (struct addrinfo* p=result;p!=NULL;p=p->ai_next) {
                if (p->ai_family == AF_INET) {
                    struct sockaddr_in* ipv4 = (struct sockaddr_in*)p->ai_addr;
                    void* addr = &(ipv4->sin_addr);
                    char ip_addr_numeric[INET6_ADDRSTRLEN];
                    inet_ntop(p->ai_family, addr, ip_addr_numeric, sizeof(ip_addr_numeric));
                    ipConverted = std::string(ip_addr_numeric);
                } 
            }
            freeaddrinfo(result);

            //Create server IP address
            bzero(&_addrServer, sizeof(_addrServer));
            _addrServer.sin_family = AF_INET;
            _addrServer.sin_port = htons(port);
            if (inet_pton(AF_INET, ipConverted.c_str(),
                &_addrServer.sin_addr.s_addr) 
                != 1
            ) {
                throw std::logic_error(
                    "inria::TransportValueUDPClient::connect: "
                    "Error converting ip address: " +
                    std::string(strerror(errno)));
            }
        }

        /**
         * Store a named value to be send to server
         * for boolean, integer or float types.
         *
         * @param name Full variable name
         * @param value Boolean, integer or float value.
         */
        void setBool(const std::string& name, bool value)
        {
            _namesBool.push_back(name);
            _valuesBool.push_back(value);
        }
        void setInt(const std::string& name, int64_t value)
        {
            _namesInt.push_back(name);
            _valuesInt.push_back(value);
        }
        void setFloat(const std::string& name, double value)
        {
            _namesFloat.push_back(name);
            _valuesFloat.push_back(value);
        }

        /**
         * Send stored values to server
         * in a UDP packet
         */
        void send()
        {
            //Get sizes
            size_t sizeBool = _namesBool.size();
            size_t sizeInt = _namesInt.size();
            size_t sizeFloat = _namesFloat.size();
            if (sizeBool + sizeInt + sizeFloat == 0) {
                return;
            }
            
            //Retrieve current time as microseconds
            auto currentTimePoint = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
                currentTimePoint.time_since_epoch());
            double timeNow = duration.count();

            //Craft packet.
            //Compute packet size.
            size_t sizePacket = 
                sizeof(double) +
                3*sizeof(uint32_t) +
                sizeBool*(NAME_MAX_LENGTH + sizeof(uint8_t)) +
                sizeInt*(NAME_MAX_LENGTH + sizeof(int64_t)) +
                sizeFloat*(NAME_MAX_LENGTH + sizeof(double));
            //Allocate memory
            unsigned char* packetData = new unsigned char[sizePacket];
            //Assign data
            size_t index = 0;
            *reinterpret_cast<double*>(&packetData[index]) = timeNow;
            index += sizeof(double);
            *reinterpret_cast<uint32_t*>(&packetData[index]) = sizeBool;
            index += sizeof(uint32_t);
            *reinterpret_cast<uint32_t*>(&packetData[index]) = sizeInt;
            index += sizeof(uint32_t);
            *reinterpret_cast<uint32_t*>(&packetData[index]) = sizeFloat;
            index += sizeof(uint32_t);
            for (size_t i=0;i<sizeBool;i++) {
                char* ptr = reinterpret_cast<char*>(&packetData[index]);
                bzero(ptr, NAME_MAX_LENGTH);
                strncpy(ptr, _namesBool[i].c_str(), NAME_MAX_LENGTH);
                index += NAME_MAX_LENGTH;
            }
            for (size_t i=0;i<sizeInt;i++) {
                char* ptr = reinterpret_cast<char*>(&packetData[index]);
                bzero(ptr, NAME_MAX_LENGTH);
                strncpy(ptr, _namesInt[i].c_str(), NAME_MAX_LENGTH);
                index += NAME_MAX_LENGTH;
            }
            for (size_t i=0;i<sizeFloat;i++) {
                char* ptr = reinterpret_cast<char*>(&packetData[index]);
                bzero(ptr, NAME_MAX_LENGTH);
                strncpy(ptr, _namesFloat[i].c_str(), NAME_MAX_LENGTH);
                index += NAME_MAX_LENGTH;
            }
            for (size_t i=0;i<sizeBool;i++) {
                *reinterpret_cast<uint8_t*>(&packetData[index]) = _valuesBool[i] ? 1 : 0;
                index += sizeof(uint8_t);
            }
            for (size_t i=0;i<sizeInt;i++) {
                *reinterpret_cast<int64_t*>(&packetData[index]) = _valuesInt[i];
                index += sizeof(int64_t);
            }
            for (size_t i=0;i<sizeFloat;i++) {
                *reinterpret_cast<double*>(&packetData[index]) = _valuesFloat[i];
                index += sizeof(double);
            }

            //Send packet
            int ret = sendto(_sockfd, packetData, sizePacket, 0, 
                (const sockaddr*)&_addrServer, sizeof(_addrServer));
            if (ret != sizePacket) {
                throw std::logic_error(
                    "inria::TransportValueUDPClient::send: "
                    "Error sending data: " +
                    std::string(strerror(errno)));
            }
            //Deallocate memory
            delete[] packetData;

            //Clear buffers
            _namesBool.clear();
            _namesInt.clear();
            _namesFloat.clear();
            _valuesBool.clear();
            _valuesInt.clear();
            _valuesFloat.clear();
        }

    private:

        /**
         * Name and value container buffer 
         * for types before sending
         */
        std::vector<std::string> _namesBool;
        std::vector<std::string> _namesInt;
        std::vector<std::string> _namesFloat;
        std::vector<bool> _valuesBool;
        std::vector<int64_t> _valuesInt;
        std::vector<double> _valuesFloat;
        
        /**
         * Socket network descriptor 
         * and server address
         */
        int _sockfd;
        struct sockaddr_in _addrServer;
};

}

#endif

