// Implementation of the ServerSocket class
#ifndef ___SERVERSOCKET_CPP___
#define ___SERVERSOCKET_CPP___

#include "ServerSocket.h"
#include "SocketException.h"


ServerSocket::ServerSocket ( int port )
{
  if ( Socket::create()<0 )
    {
      throw SocketException ( "Could not create server socket." );
    }

  if ( ! Socket::bind ( port ) )
    {
      throw SocketException ( "Could not bind to port." );
    }

  if ( ! Socket::listen() )
    {
      throw SocketException ( "Could not listen to socket." );
    }

}

ServerSocket::~ServerSocket()
{
}


const ServerSocket& ServerSocket::operator << ( const std::string& s ) const
{
  if ( ! Socket::send ( s ) )
    {
      throw SocketException ( "Could not write to socket." );
    }

  return *this;

}


const ServerSocket& ServerSocket::operator >> ( std::string& s ) const
{
  if ( ! Socket::recv ( s ) )
    {
      throw SocketException ( "Could not read from socket." );
    }

  return *this;
}

int ServerSocket::accept ( ServerSocket& sock )
{
  int socketnum;
  socketnum= Socket::accept ( sock );
  if ( socketnum<0 )
    {
      throw SocketException ( "Could not accept socket." );
    }
}

int ServerSocket::get()
{
 return Socket::getSock();
}

#endif

