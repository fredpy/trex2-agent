#ifndef H_ControlInterface
# define H_ControlInterface

# include <trex/transaction/TeleoReactor.hh>

# include <boost/thread.hpp>
# include <boost/thread/recursive_mutex.hpp>

namespace TREX {
  namespace LSTS {
    
    
    /** @brief Simple goal posting interface for TREX
     *
     * This reactor provides a simple interface to post XML goals to TREX.
     * It creates a unix named pipe and listen to it for parsing new messages 
     * coming. Whenever such message properly parse as a goal it will post it
     * to the corresponding timeline.
     *
     * @author Frederic Py
     */
    class ControlInterface :public TREX::transaction::TeleoReactor {
    public:
      ControlInterface(TREX::transaction::TeleoReactor::xml_arg_type arg);
      ~ControlInterface();
      
      bool is_running() const;
      void stop();
      
      /** @brief Porcess new message(s)
       *
       * @param[in] msg A message
       *
       * Parse the contetn of @p msg and queue any goal extracted from it.
       */
      void proccess_message(std::string const &msg);

    private:
      class thread_proxy {
      public:
        thread_proxy(thread_proxy const &other);
        ~thread_proxy();
        
        void operator()();
        
      private:
        thread_proxy(ControlInterface *me);
                     
        ControlInterface *m_reactor;
        
        friend class ControlInterface;
      };
      
      friend class thread_proxy;
      
      void handleInit();
      void handleTickStart();
      bool synchronize();
      
      void add_goal(TREX::transaction::goal_id const &g);
      bool next_goal(TREX::transaction::goal_id &g);
      
      void run();
      
      /** @brief Name of the fifo pipe
       *
       * @return The file name to access the new pipe
       */
      std::string fifo_name() const;
      
      /** @brief Create the fifo pipe 
       *
       * Create the new fifo pipe and open it. 
       *
       * @trow TREX::utils::ErrnoExcept system error while creating the 
       * fifo pipe.
       */
      void create_fifo();
      void destroy_fifo();
      bool is_open() const;
      
      /** @brief Wait for data from fifo
       *
       * @param[out] buff A buffer
       * @param[in] buff_size The size of @p buff
       * @param[in] us_timer micro seconds until time out
       *
       * Wait for new data in m_fifo until @p us_timer delay. If any data is 
       * available the first @p buff_size bytes are writtedn in @p buff
       *
       * @return the number of bytes received
       */
      size_t retrieve_from_fifo(char *buff, size_t buff_size, int us_timer);
      
      typedef boost::recursive_mutex mutex_type;
      typedef mutex_type::scoped_lock scoped_lock;

      mutable mutex_type m_mutex;
      std::auto_ptr<boost::thread> m_thread;
      
      
      bool m_running;
      std::list<TREX::transaction::goal_id> m_pending_goals;
      
      /** @brief fifo pipe file descriptor
       *
       * The file descriptor of the fifo pipe this reactor listen to or 
       * @c 0 if no pipe has been created yet.
       */
      int m_fifo;
      
      std::string log_message(std::string const &content);
      static TREX::utils::SharedVar<size_t> s_id;

    }; // TREX::LSTS::ControlInterface
    
  } // TREX::LSTS
} // TREX

#endif // H_ControlInterface
