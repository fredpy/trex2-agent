/* -*- C++ -*- */
/** @file "SingletonWrapper.tcc"
 * @brief SingletonWrapper implementation
 *
 * @author Frederic Py <fpy@mbari.org>
 * @ingroup utils 
 */
#ifndef In_H_SingletonWrapper
# error "Cannot include tcc file outside of its corresponding header"
#else 

#include <typeinfo>

namespace TREX {
  namespace utils {

    namespace internal {

      template<typename Ty>
      struct swrapper_factory :public sdummy_factory {
	SingletonDummy *create() const {
	  return new SingletonWrapper<Ty>;
	}
      }; 

    }

    // statics :
    
    template<typename Ty>
    Ty *SingletonWrapper<Ty>::attach() {
      SingletonWrapper<Ty> *
	me = static_cast<SingletonWrapper<Ty> *>(internal::SingletonDummy::attach(name(), 
                                                                                  internal::swrapper_factory<Ty>()));
      return &(me->m_value);
    }

    template<typename Ty>
    void SingletonWrapper<Ty>::detach() {
      internal::SingletonDummy::detach(name());
    }

    template<typename Ty>
    std::string SingletonWrapper<Ty>::name() {
      return typeid(Ty).name();
    }

    // structors :

    template<typename Ty>
    SingletonWrapper<Ty>::SingletonWrapper() {}

    template<typename Ty>
    SingletonWrapper<Ty>::~SingletonWrapper() {}

  }
}

#endif  
