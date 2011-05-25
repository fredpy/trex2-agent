/** @file "utils/base/Hashable.hh"
 * @brief TR1 hashing utilities
 *
 * This file provides some utilities to ease the
 * definition of th has function for a class.
 *
 * @note This implementation is currently based on boost
 * TR1 compatibility hash definition
 *
 * @author Frederic Py <fpy@mbari.org>
 * @ingroup utils
 */   
#ifndef H_Hashable
# define H_Hashable 

# include <cstddef>
# include <boost/functional/hash.hpp>  

namespace TREX {
  namespace utils {
    class Hashable;
	  
		/** @brief Hashing proxy for boost::hash
		 *
		 * @param x Insance to hash
		 *
		 * This method is used internally by boost::hash functor to
		 * compute the hash value
		 *
		 * @return The hash value of @a x
		 *
		 * @sa std::size_t Hashable::hash() const
		 * @relates TREX::utils::Hashable
		 * @ingroup utils
		 */
 		size_t hash_value(TREX::utils::Hashable const &x);
		
    
    /** @brief Abstract class that supports hash
     *
     * This class provides an easy way to support hash function
     * (and consequently TR1 collections) for classes.
     *
     * To do so one just need to inherit from this class and
     * implements the hash() method accordingly
     *
     * @author Frederic Py <fpy@mbari.org>
     * @ingroup utils
     */	
    class Hashable {
    public:
      /** @brief Constructor */
      Hashable() {}
      /** @brief Destructor */
      virtual ~Hashable() {}
      
    protected:
      /** @brief hashing method
       *
       * This method is called by the TR1` hash functor to get
       * the hash value of current instance.
       *
       * @return computed hash value
       */
      virtual size_t hash() const =0;
			
			friend std::size_t hash_value(TREX::utils::Hashable const &x);
    }; // TREX::utils::Hashable
    
		
		inline std::size_t hash_value(TREX::utils::Hashable const &x) {
			return x.hash();
		}
		
  } // boost::utils
} // boost

#endif // H_Hashable
