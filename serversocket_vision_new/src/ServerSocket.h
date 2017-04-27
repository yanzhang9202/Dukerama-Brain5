// Definition of the ServerSocket class
#ifndef ServerSocket_H
#define ServerSocket_H

#include "Socket.cpp"

class ServerSocket : private Socket
{
 public:

  ServerSocket ( int port );
  ServerSocket (){};
  virtual ~ServerSocket();
  int get();  

  const ServerSocket& operator << ( const std::string& ) const;
  const ServerSocket& operator >> ( std::string& ) const;

  int accept ( ServerSocket& );
  

};


#endif
