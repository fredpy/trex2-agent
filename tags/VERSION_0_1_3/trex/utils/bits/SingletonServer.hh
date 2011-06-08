/** @file "SingletonServer.hh"
 * @brief Defintion of the SingletonServer class
 *
 * This header is for internal use and define the
 * SingletonServer class. This class is the centralized
 * place where all the singletons are maintained
 *
 * @author Frederic Py <fpy@mbari.org>
 * @ingroup utils 
 */
#ifndef H_SingletonServer
# define H_SingletonServer

# include <boost/thread/recursive_mutex.hpp>

# include <map>

# include "SingletonDummy.hh"

namespace TREX {
  namespace utils {
    namespace internal {

      /** @brief Singleton central manager
       *
       * This class is the container where all the phoenix
       * singleton acccessed through SingletonUse are
       * maintained.
       *
       * All its methods are private as no other class other  than
       * SingletonDummy should manipulate it.
       *
       * @author Frederic Py <fpy@mbari.org>
       * @ingroup utils
       */
      class SingletonServer : boost::noncopyable {
      private:
	SingletonServer();
	~SingletonServer();

	static SingletonServer &instance();

	SingletonDummy *attach(std::string const &name, 
                               sdummy_factory const &factory);
	bool detach(std::string const &name);

	typedef std::map<std::string, SingletonDummy *> single_map;
	
	single_map m_singletons;
	
	static SingletonServer *s_instance;

	typedef boost::recursive_mutex mutex_type;
	typedef mutex_type::scoped_lock lock_type;

	static mutex_type &sing_mtx();

	friend class SingletonDummy;
      }; // TREX::utils::internal::SingletonServer

    }
  }
}

#endif // H_SingletonServer
