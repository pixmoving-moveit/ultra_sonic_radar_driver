#pragma once

#include <iostream>

#define MC_ERROR   std::cout << "\033[1m\033[31m"  // bold red
#define MC_WARNING std::cout << "\033[1m\033[33m"  // bold yellow
#define MC_INFO    std::cout << "\033[1m\033[32m"  // bold green
#define MC_INFOL   std::cout << "\033[32m"         // green
#define MC_DEBUG   std::cout << "\033[1m\033[36m"  // bold cyan
#define MC_END    "\033[0m" << std::endl

#define MC_TITLE   std::cout << "\033[1m\033[35m"  // bold magenta
#define MC_MSG     std::cout << "\033[1m\033[37m"  // bold white

