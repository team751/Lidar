#include "../inc/roborio_connection.h"

#include <arpa/inet.h>
#include <netdb.h>

#include <cstring>

#include "../inc/constants.h"

// Opens a connection to the server if one doesn't already exist
// Returns (boolean): returns true if the connection was successfully
// established or false if an error occurred
bool RoboRIOConnection::start(std::string host, std::string port) {
  // Check preconditions
  if (isOpen()) return false;

  // Create a new socket with IPv4 (AF_INET) that supports datagrams
  // (UDP packets) using the default protocol (0 means use default)
  int roboRIOSocket = socket(AF_INET, SOCK_DGRAM, 0);

  // Set SO_BROADCAST (required by OS)
  // (see http://stackoverflow.com/questions/16217958/why-do-we-need-socketoptions-so-broadcast-to-enable-broadcast)
  int bcast = 1;
  if (setsockopt(roboRIOSocket, SOL_SOCKET, SO_BROADCAST, &bcast, sizeof(bcast)) < 0) {
    return false;
  }

  // Set timeout value with default from the constants file
  if (0 > setsockopt(roboRIOSocket, SOL_SOCKET, SO_RCVTIMEO, &Constants::defaultTimeoutValue, sizeof(Constants::defaultTimeoutValue))) {
    return false;
  }

  // Store the socket
  this->currentSocket = roboRIOSocket;

  // Set host and port variables
  this->host = host.c_str();
  this->port = port.c_str();

  // Set open variable
  this->connectionOpen = true;

  return true;
}

// Closes the connection to the server
// Returns (boolean): returns true if the connection was successfully
// closed
void RoboRIOConnection::stop() {
  this->host = NULL;
  this->port = NULL;

  this->connectionOpen = false;
}

// Returns (boolean): returns true if the a connection to the server is
// already open
bool RoboRIOConnection::isOpen() {
  return connectionOpen;
}

// Sends a datapoint to the RoboRIO
// Returns (boolean): returns true if the message successfully sent
bool RoboRIOConnection::send(TotePose totePacket) {
  // Check preconditions
  if (!isOpen()) return false;
  if (host == NULL) return false;
  if (port == NULL) return false;

  // Setup variables
  struct hostent* hostEntry; // Stores the host entry of the roboRIO
  struct sockaddr_in remoteAddress; // Store the address of the roboRIO

  // Set the host entry
  hostEntry = gethostbyname(this->host);

  // Configure remote address
  remoteAddress.sin_family = AF_INET; // IPv4
  remoteAddress.sin_port = htons(atoi(port)); // Port number
  remoteAddress.sin_addr = *(struct in_addr *)hostEntry->h_addr; // Address from hostEntry
  memset(remoteAddress.sin_zero, 0, sizeof(remoteAddress.sin_zero));

  // Generate a message from the packet
  std::string message = totePacket.formattedMessage();

  // Send message and stores the number of characters sent
  int charactersSent = sendto(this->currentSocket, message.c_str(), message.length(), 0, (struct sockaddr *)&remoteAddress, sizeof(remoteAddress));

  return true;
}
