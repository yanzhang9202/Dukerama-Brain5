// This driver code is referred to the "Serial Example Code" part in Micro Maestro Controller User Manual
// Created by Yan Zhang, on Sep 6, 2015
#ifndef MAESTRO_SERIAL_DRIVER_CPP
#define MAESTRO_SERIAL_DRIVER_CPP
#endif
// Uses POSIX functions to send and receive data from a Maestro.
// NOTE: The Maestro's serial mode must be set to "USB Dual Port".
// NOTE: You must change the 'const char * device' line below.

#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <termios.h>

// Connect to the servo
int maestroConnect()
{
  // Open the Maestro's virtual COM port.
  const char * device = "/dev/ttyACM0"; // Linux
  int fd = open(device, O_RDWR | O_NOCTTY);
  if (fd == -1)
  {
    perror(device);
    fprintf(stderr, "error opening");
    exit(1);
  }

  struct termios options;
  tcgetattr(fd, &options);
  options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
  options.c_oflag &= ~(ONLCR | OCRNL);
  tcsetattr(fd, TCSANOW, &options);

  return fd;
}

// Gets the position of a Maestro channel.
// See the "Serial Servo Commands" section of the user's guide.
int maestroGetPosition(int fd, unsigned char channel)
{
  unsigned char command[] = {0x90, channel};
  if(write(fd, command, sizeof(command)) == -1)
  {
    perror("error writing");
    return -1;
  }

  unsigned char response[2];
  if(read(fd,response,2) != 2)
  {
    perror("error reading");
    return -1;
  }
  return response[0] + 256*response[1];
}

// Sets the target of a Maestro channel.
// See the "Serial Servo Commands" section of the user's guide.
// The units of 'target' are quarter-microseconds.
int maestroSetTarget(int fd, unsigned char channel, unsigned short target)
{
  unsigned char command[] = {0x84, channel, target & 0x7F, target >> 7 & 0x7F}; 
  if (write(fd, command, sizeof(command)) == -1)
  {
    perror("error writing");
    return -1;
  }
  return 0;
}

bool GetMovingState(int fd, unsigned char channel){
  unsigned char command[] = {0x93};//, channel};
  int written=write(fd,command,sizeof(command));
  if (written==-1){
    printf("Error sending state check\n");
    return false;
  }

  unsigned char response[2];
  
  int isread=read(fd,response,2);

  return (response[0]==1);
}

int maestroGoHome(int fd, unsigned char channel)
{
//  unsigned char command[] = {0xA2}; 
//  if (write(fd, command, sizeof(command)) == -1)
//  {
//    perror("error going home.\n");
//    return -1;
//  }
//  printf("servo going home.\n");
//  return 0;
  maestroSetTarget(fd, channel, 1500*4);
  return 0;
}

int maestroSetSpeed(int fd, unsigned char channel, unsigned short speed)
{
  unsigned char command[] = {0x87, channel, speed & 0x7F, speed >> 7 & 0x7F}; 
  if (write(fd, command, sizeof(command)) == -1)
  {
    perror("error writing");
    return -1;
  }
  return 0;
}



