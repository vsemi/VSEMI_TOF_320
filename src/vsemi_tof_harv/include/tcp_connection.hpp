#ifndef __ESPROS_TCPCONNECTION_H__
#define __ESPROS_TCPCONNECTION_H__

#include <boost/asio.hpp>

using boost::asio::ip::tcp;

namespace espros {

class TcpConnection {
  static const int MARKER_SIZE = 4;
  static const int ACK_BUF_SIZE = 128;
  static constexpr const char* PORT = "50660";
  static constexpr const char* HOST = "10.10.31.180";
  //static constexpr const char* HOST = "192.168.0.80";
  static constexpr const char* END_MARKER = "\xff\xff\x55\xaa";
  static constexpr const char* START_MARKER = "\xff\xff\xaa\x55";

public:
  enum State {
    STATE_CONNECTING,
    STATE_DISCONNECTED,
    STATE_CONNECTED,
    STATE_CLOSING,
    STATE_WAIT_ACK
  };

  TcpConnection(boost::asio::io_service &);
  ~TcpConnection();

  void sendCommand(const std::vector<uint8_t> &);

private:
  mutable State state, previousState;
  tcp::socket socket;
  tcp::resolver resolver;

  void connect();
  void disconnect();
  void waitAck();
  void updateState(State) const;
  void revertState() const;
  bool isConnected() const;
  bool isDisconnected() const;
};

} //end namespace espros

#endif // __ESPROS_TCPCONNECTION_H__
