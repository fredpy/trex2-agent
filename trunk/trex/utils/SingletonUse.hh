/* -*- C++ -*-
 * $Id$
 */
/** @file "SingletonUse.hh"
 * @brief Define the access class to a phoenix singleton
 *
 * @author Frederic Py <fpy@mbari.org>
 * @ingroup utils
 */
#ifndef H_SingletonUse
# define H_SingletonUse

namespace TREX {
  namespace utils {

    /** @brief Singleton accessor
     * @tparam Ty type of the singleton
     *
     * This class is used to access to a phoenix singleton.
     * It also manages the singleton creation and lifetime
     * through reference counting
     *
     * @author Frederic Py <fpy@mbari.org>
     * @ingroup utils
     */
    template<typename Ty>
    struct SingletonUse {
    public:
      /** @brief Constructor
       *
       * Create a new instance referring to the singleton @a Ty. If
       * this singleton does not already exist it is created using
       * the default constructor of @a Ty. On any case the reference
       * counter for the singleton @a Ty is incremented by 1.
       */
      SingletonUse();
      /** @overload SingletonUse() */
      SingletonUse(SingletonUse<Ty> const &other);
      /** @brief Destructor
       * Send a notification to the singleton mmanager that one
       * instance less is accessing to the singleton @a Ty if no
       * more instance are refering to it them this singleton will
       * be destroyed
       */
      ~SingletonUse();

      /** @brief Singleton instance
       * @return a reference to the singleton @a Ty
       * @{
       */
      Ty &instance() const;
      Ty &operator*() const;
      Ty *operator->() const;
      /** @} */
    private:

      Ty *m_instance; //!< Singleton_reference
    }; // TREX::utils::SingletonUse<>
  }
}    
# define In_H_SingletonUse
#  include "bits/SingletonUse.tcc"
# undef  In_H_SingletonUse

#endif // H_SingletonUse
