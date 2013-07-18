#include <fstream>
#include <iostream>
#include <vector>

#include <unistd.h>
#include <errno.h>

#include <trex/utils/platform/chrono.hh>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/optional.hpp>

#define BUFF_SIZE 4096

int main(int argc, char **argv) {
  if( argc<2 ) {
    std::cerr<<"usage: "<<argv[0]<<" <pid>"<<std::endl;
    exit(-1);
  }
  
  // build path name for the stat file
  std::string path(argv[1]);
  path = "/proc/"+path+"/stat";
  
  // Get the clk_tck
  int clock_ticks = sysconf(_SC_CLK_TCK);
  if( clock_ticks<0 ) {
    std::cerr<<"Failed to get system clock ticks: "<<strerror(errno)<<std::endl;
    exit(1);
  } else
    std::cerr<<"System running at "<<clock_ticks<<"Hz"<<std::endl;
  
  unsigned long long ns_fact = 1000000000ull;
  ns_fact /= clock_ticks;
  
  char stats_buff[BUFF_SIZE+1];
  stats_buff[BUFF_SIZE] = '\0';
  boost::optional<CHRONO::nanoseconds> prev_date, prev_stat;

  
  while(1) {
    std::ifstream in(path.c_str());
    
    CHRONO::system_clock::time_point st = CHRONO::system_clock::now();
  
    if( in.bad() )
      break;
    else {
      in.getline(stats_buff, BUFF_SIZE);
      in.close();
      
      std::vector<std::string> args;
      boost::algorithm::split(args, stats_buff, boost::algorithm::is_space());
      
      unsigned long long
        utime = boost::lexical_cast<unsigned long long>(args[13]),
        stime = boost::lexical_cast<unsigned long long>(args[14]);
      
      utime *= ns_fact;
      stime *= ns_fact;
    
      
      CHRONO::nanoseconds
        rt = CHRONO::duration_cast<CHRONO::nanoseconds>(st.time_since_epoch()),
        ut(utime), st(stime);
      
      unsigned long long pcpu = 0;
      
      if( prev_date ) {
        CHRONO::nanoseconds dt = rt, dts = ut+st;
        dt -= *prev_date;
        if( dt>CHRONO::nanoseconds::zero() ) {
          dts -= *prev_stat;
          pcpu = dts.count()/dt.count();
        }
      } else
        std::cout<<"time_ns, utime_ns, stime_ns, pcpu, state"<<std::endl;
      std::cout<<rt.count()<<", "<<ut.count()<<", "<<st.count()
      <<", "<<pcpu<<", "<<args[2]<<std::endl;
      prev_date = rt;
      prev_stat = ut+st;
      struct timespec rq, rm;
      rq.tv_sec = 0;
      rq.tv_nsec = CHRONO::duration_cast<CHRONO::nanoseconds>(CHRONO::milliseconds(100)).count(); // wake up every 100ms or so
      nanosleep(&rq, &rm);
    }
    
    
  }
  std::cerr<<"EOP"<<std::endl;
  return 0;
}

