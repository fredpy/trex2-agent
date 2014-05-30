#ifndef H_trex_utils_priority_strand
# define H_trex_utils_priority_strand

# include "platform/cpp11_deleted.hh"

# include <boost/asio/strand.hpp>
# include <boost/optional.hpp>
# include <boost/thread/future.hpp>
# include <boost/utility/result_of.hpp>

# include <queue>

namespace TREX {
  namespace utils {
 
    class priority_strand;
    
    namespace details {
      
      template<typename Fn>
      struct task_helper {
        typedef typename boost::result_of<Fn()>::type return_type;
        typedef boost::shared_future<return_type>     future;
      }; // TREX::utils::details::task_helper<>
      
      class priority_task :boost::noncopyable {
      public:
        typedef size_t priority_type;
        bool operator< (priority_task const &other) const;
        
      protected:
        priority_task();
        priority_task(priority_type p);
        virtual ~priority_task() {}

        virtual void execute() =0;
        
      private:
        boost::optional<priority_type> m_priority;
        
        friend class priority_strand;
      }; // TREX::utils::details::priority_task
      
    } // TREX::utils::details
    
    class priority_strand {
    public:
      
      class task {
      public:
        typedef size_t priority;
        
        bool operator< (task const &other) const;
        
      protected:
        task() {}
        task(priority p):m_level(p) {}
        virtual ~task() {}
        
        virtual void execute() =0;
        
      private:
        boost::optional<priority> m_level;
        
        friend priority_strand;
      };
      
      typedef task::priority priority_type;
    
      priority_strand(boost::asio::io_service &io);
      ~priority_strand();
      
      template<typename Fn>
      typename details::task_helper<Fn>::future post(Fn f);
      template<typename Fn>
      typename details::task_helper<Fn>::future post(Fn f, priority_type p);
      template<typename Fn>
      void async(Fn f);
      template<typename Fn>
      void async(Fn f, priority_type p);
      
    private:
      struct tsk_cmp {
        bool operator()(task *a, task *b) const;
      };

      typedef std::priority_queue<task *,std::vector<task *>, tsk_cmp> task_queue;

      boost::asio::strand m_strand;
      task_queue          m_tasks;
      
      
      void enqueue(task *tsk);
    
      void enqueue_sync(task *tsk);
      void dequeue_sync();
    }; // TREX::utils::priority_queue

# define IN_trex_utils_priority_strand
#  include "bits/priority_strand.tcc"
# undef IN_trex_utils_priority_strand

  } // TREX::utils
} // TREX

#endif // H_trex_utils_priority_strand