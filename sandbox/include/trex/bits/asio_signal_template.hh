#ifdef ASIO_SIGNAL_NUM_ARGS

# include <boost/preprocessor/repetition/enum_params.hpp>

# define ASIO_SIGNAL_PARAM(J,I,D) BOOST_PP_CAT(A,I) BOOST_PP_CAT(arg,I)
# define ASIO_SIGNAL_PARAMS BOOST_PP_ENUM(ASIO_SIGNAL_NUM_ARGS,ASIO_SIGNAL_PARAM,BOOST_PP_EMPTY)
# define ASIO_SIGNAL_ARGS BOOST_PP_ENUM_PARAMS(ASIO_SIGNAL_NUM_ARGS,arg)

namespace trex {

  template<BOOST_PP_ENUM_PARAMS(ASIO_SIGNAL_NUM_ARGS,typename A)>
  class asio_signal<void (BOOST_PP_ENUM_PARAMS(ASIO_SIGNAL_NUM_ARGS,A))>
  :public details::asio_signal_base<void (BOOST_PP_ENUM_PARAMS(ASIO_SIGNAL_NUM_ARGS,A))>
# if ASIO_SIGNAL_NUM_ARGS==1
  ,public std::unary_function<A0,void>
# elif ASIO_SIGNAL_NUM_ARGS==2
  ,public std::binary_function<A0,A1,void>
# endif // ASIO_SIGNAL_NUM_ARGS
  {
    typedef details::asio_signal_base<void (BOOST_PP_ENUM_PARAMS(ASIO_SIGNAL_NUM_ARGS,A))> base_type;
  public:
    static unsigned const arity=ASIO_SIGNAL_NUM_ARGS;
    
    explicit asio_signal(::boost::asio::io_service &io):base_type(io) {}
    ~asio_signal() {}
    
    void operator()(ASIO_SIGNAL_PARAMS) {
      this->m_signal(ASIO_SIGNAL_ARGS);
    }
    
  }; // TREX::utils::asio_signal<void (...)>
  
} // TREX

# undef ASIO_SIGNAL_ARGS
# undef ASIO_SIGNAL_PARAMS
# undef ASIO_SIGNAL_PARAM
# undef ASIO_SIGNAL_NUM_ARGS

#endif // ASIO_SIGNAL_NUM_ARGS