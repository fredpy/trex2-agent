#ifndef H_ControlInterface
# define H_ControlInterface

# include <trex/transaction/TeleoReactor.hh>

# include <boost/thread.hpp>
# include <boost/thread/recursive_mutex.hpp>

# include <boost/bimap.hpp>

# include <set>

# include "SharedEnvironment.hh"

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
      /** @brief Constructor 
       *
       * @param[in] arg XML reactor definition
       *
       * XML format :
       * @code 
       *  <GoalPipe name="<reactorname>" lookahead="<lookahead>" 
       *            latency="<latency>" log="<logflag>" />
       * @endcode 
       *
       * @note As this reactor does not provide internal timelin
       *   the lookahaded and latency are meaningless and can be
       *   set to 0.
       */
      ControlInterface(TREX::transaction::TeleoReactor::xml_arg_type arg);
      /** @brief Destructor 
       *
       * Terminates the listener thread, and destory this 
       * instance. As a resulult the FIFO unqix pipe should be 
       * destroyed.  
       */
      ~ControlInterface();
      
      /** @brief Test if running
       *
       * Check if the listener thread of this reactor is 
       * currently running.
       *
       * @retval true I f the thread is running
       * @retval false otherwise
       */
      bool is_running() const;
      /** @brief Stop the listener thread
       *
       * Stops the thread listenning to messages.
       */
      void stop();
      
      /** @brief Process new message(s)
       *
       * @param[in] msg A message
       *
       * Parse the content of @p msg and queue any goal extracted 
       * from it.
       */
      void proccess_message(std::string const &msg);

    private:

      void notify(TREX::transaction::Observation const &obs);

      /** @brief Message listeenign thread.
       *
       * The thread that listen and gather  messages coming 
       * from the fifo queue.
       *
       * @relates ControlInterface
       * @uathor Frederic Py 
       */ 
      class thread_proxy {
      public:
        /** @brief Copy constructor
         *
         * @param[in] other Another instance.
         *
         * Create a copy of @p other that refers to the same
         * ControlInterface. This constructor is necessary as 
         * boost thread passes thread execution class by value
         */ 
        thread_proxy(thread_proxy const &other);
        /** @brief Destructor */
        ~thread_proxy();
        
        /** @brief Thread execution method
         *
         * The code executed by the thread. This method is just a
         * proxy to ControlInterface::run() 
         *
         * @sa ControlInterface::run()
         */ 
        void operator()();
        
      private:
        /** @brief Constructor
         *
         * @param[in] me A pointer to the caller
         *
         * Create a new instance that refers to @p me
         */
        thread_proxy(ControlInterface *me);


        ControlInterface *m_reactor;
        
        friend class ControlInterface;
      };
      
      friend class thread_proxy;
      
      /** @brief Reactor initialization
       *
       * Initialize the reactor within the agent. This call will:
       * @li create the fifo queue to listen to
       * @li spawn a thread listening to this queue
       * @li set the agent ControlInterface singleton to this 
       *    instance 
       */ 
      void handleInit();
      void handleTickStart();
      bool synchronize();
      void newPlanToken(TREX::transaction::goal_id const &t);
      void cancelledPlanToken(TREX::transaction::goal_id const &t);
      

      /** @brief Add a goal
       *
       * @param[in] g A goal 
       * 
       * Add @p g to the pending goals queue
       *
       * @post the pending goal queue is not empty
       */
      void add_goal(TREX::transaction::goal_id const &g,
                    boost::optional<std::string> const &id);
      void add_recall(std::string const &id);

      bool next(std::set<TREX::transaction::goal_id> &l,
                TREX::transaction::goal_id &g);

      
      /** @brief Thead listening execution loop
       *
       * The loop executed by the queue listening thread. This 
       * loop ewaits for new message in the fifo, parse them as 
       * goals and stor the resulting goal in the pending goals 
       * queue
       */
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
       * @trow TREX::utils::ErrnoExcept system error while creating 
       * the fifo pipe.
       */
      void create_fifo();
      /** @brief Destroy fifo queue
       *
       * Close the fifo queue created by this reactor and destroy
       * it
       */ 
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
      std::set<TREX::transaction::goal_id> m_pending_goals,
        m_pending_recalls;
      
      typedef boost::bimap<std::string, TREX::transaction::goal_id> goal_map; 
      goal_map m_goals;
      
      bool m_need_fifo;
      /** @brief fifo pipe file descriptor
       *
       * The file descriptor of the fifo pipe this reactor listen to or 
       * @c 0 if no pipe has been created yet.
       */
      int m_fifo;
      
      /**
       * @brief pointer to singleton class
       */
      TREX::utils::SingletonUse<SharedEnvironment> m_env;

      std::string log_message(std::string const &content);
      static TREX::utils::SharedVar<size_t> s_id;
      
      // TODO : put a bimap to associate goals to their id

    }; // TREX::LSTS::ControlInterface
    
  } // TREX::LSTS
} // TREX

#endif // H_ControlInterface
