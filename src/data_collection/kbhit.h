#pragma once

#include <termios.h>
#include <sys/ioctl.h>

#include <unistd.h>

inline bool kbhit() {
  termios term;
  tcgetattr(STDIN_FILENO, &term);
  termios term2 = term;
  term2.c_lflag &= ~ICANON;
  tcsetattr(STDIN_FILENO, TCSANOW, &term2);
  int byteswaiting = 0;
  ioctl(STDIN_FILENO, FIONREAD, &byteswaiting);
  tcsetattr(STDIN_FILENO, TCSANOW, &term);
  return byteswaiting > 0;
}
