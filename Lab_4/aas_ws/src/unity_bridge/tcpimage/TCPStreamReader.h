#ifndef TCPSTREAMREADER_H
#define TCPSTREAMREADER_H

#include <string>
#include <stdint.h>
#include <libsocket/inetserverstream.hpp>
#include <libsocket/select.hpp>
#include <memory>
#include <iostream>

class TCPStreamReader {
    std::string m_host;
    std::string m_port;
    libsocket::inet_stream_server m_server;
    libsocket::inet_stream_server* m_ready_server;
    libsocket::inet_stream *m_server_stream;
    libsocket::selectset<libsocket::inet_stream_server> m_server_set;
    libsocket::selectset<libsocket::inet_stream_server>::ready_socks m_readypair;
    bool m_good;
public:
    TCPStreamReader(std::string const& host, std::string const& port);
    virtual ~TCPStreamReader();
    void WaitConnect();
    bool Good();
    void Shutdown();
    int ReadInt();
    uint32_t ReadUInt();
    float ReadFloat();
    std::shared_ptr<uint8_t> ReadBytes(const size_t num_bytes);
};

#endif 
