/** @file "utils/base/IOstreamable.hh"
 * @brief Standard streams support utilities
 *
 * This file provides some utilities to ease the
 * support of standard stream from user defined classes
 *
 * @author Frederic Py <fpy@mbari.org>
 * @ingroup utils
 */   
#ifndef H_IOstreamable 
# define H_IOstreamable

# include <iostream>

namespace TREX {
  namespace utils {
    
    class ostreamable;
    class istreamable;

  } // TREX::utils
} // TREX

namespace std {
	
  ostream &operator<<(ostream &, TREX::utils::ostreamable const &);
  istream &operator>>(istream &, TREX::utils::istreamable &);

} // std

namespace TREX {
  namespace utils {

    /** @brief standard output stream support class
     *
     * This class defines a generic interface to C++ standard
     * output streams.
     *
     * To support the standard << operator one just need to inherit
     * from this class and implements the @c print_to method.
     *
     * @author Frederic Py <fpy@mbari.org>
     * @ingroup utils
     */
    class ostreamable {
    public:
      /** @brief Constructor. */
      ostreamable() {}
      /** @brief Destructor. */
      virtual ~ostreamable() {}
      
    protected:
      /** @brief output method
       *
       * @param[in,out] out An output stream
       *
       * This method is internally called by the standard output
       * stream operator to write the value of current instance into @p out
       *
       * @sa std::ostream &operator<<(std::ostream &, ostreamable const &)
       */
      virtual std::ostream &print_to(std::ostream &out) const =0;

      friend std::ostream &std::operator<<(std::ostream &out, 
																					 TREX::utils::ostreamable const &x);
    }; // TREX::utils::ostreamable

    /** @brief standard input stream support class
     *
     * This class defines a generic interface to C++ standard
     * input streams.
     * 
     * To support the standard >> operator one just need to inherit
     * from this class and implements the @c read_from method.
     *
     * @author Frederic Py <fpy@mbari.org>
     * @ingroup utils
     */
    class istreamable {
    public:
      /** @brief Constructor */
      istreamable() {}
      /** @brief Destructor */
      virtual ~istreamable() {}

    protected:
      /** @brief Input method
       *
       * @param[in] in An input stream
       *
       * This method is internally called by the standard input
       * stream operator to read anew value of this intance from @p in 
       *
       * @sa std::istream &std::operator<<(std::istream &, istreamable &)
       */
      virtual std::istream &read_from(std::istream &in) =0;

      friend std::istream &std::operator>>(std::istream &in, istreamable &x);
    };
  }
}

namespace std {
  /** @brief Standard output stream operator
   * @param[in,out] out an output stream
   * @param[in] x an @c ostreamable instance
   *
   * Prints the content of @p x into @p out
   * 
   * @return @p out after the operation
   *
   * @relates TREX::utils::ostreamable
   * @sa std::ostream &TREX::utils::ostreamable::print_to(std::ostream &) const
   */
  inline ostream &operator<<(ostream &out, 
			     TREX::utils::ostreamable const &x) {
    return x.print_to(out);
  }
  
  /** @brief Standard input stream operator
   *
   * @param[in,out] in an input stream
   * @param[out] x an istreamable instance
   *
   * Reads an instreamable in @p in and stores it in @p x
   *
   * @return @p in after the operation
   *
   * @relates TREX::utils::istreamable
   * @sa std::istream &TREX::utils::istreamable::read_from(std::istream &)
   */
  inline istream &operator>>(istream &in, 
			     TREX::utils::istreamable &x) {
    return x.read_from(in);
  }

} // std

#endif // H_IOstreamable
