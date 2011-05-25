/* -*- C++ -*- */
/** @file SingletonUse.tcc
 * @brief SingletonUse implementation
 *
 * @author Frederic Py <fpy@mbari.org>
 * @ingroup utils
 */
#ifndef In_H_SingletonUse
# error "Cannot include tcc files outside of their corresponding header"
#else

#include "SingletonWrapper.hh"

namespace TREX {
  namespace utils {

    template<typename Ty>
    SingletonUse<Ty>::SingletonUse()
      :m_instance(SingletonWrapper<Ty>::attach()) {}

    template<typename Ty>
    SingletonUse<Ty>::SingletonUse(SingletonUse<Ty> const &)
      :m_instance(SingletonWrapper<Ty>::attach()) {}

    template<typename Ty>
    SingletonUse<Ty>::~SingletonUse() {
      m_instance = 0x0;
      SingletonWrapper<Ty>::detach();
    }

    template<typename Ty>
    Ty &SingletonUse<Ty>::instance() const {
      return *m_instance;
    }

    template<typename Ty>
    Ty &SingletonUse<Ty>::operator*() const {
      return instance();
    }

    template<typename Ty>
    Ty *SingletonUse<Ty>::operator->() const {
      return &instance();
    }
    
  }
}

#endif 
