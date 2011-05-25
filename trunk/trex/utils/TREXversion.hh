#ifndef H_TREXversion
# define H_TREXversion

# include <string> 

namespace TREX {
    
  struct version {
    static unsigned long number();

    static unsigned short major();
    static unsigned short minor();
    static unsigned short release();

    static std::string str();
    
  }; // TREX::version
  
} // TREX

#endif // H_TREXversion
