/** @file "trex/utils/bits/Factory.tcc"
 * @brief Factory class implementation
 *
 * This file provide the implementation of different methods of the
 * TREX::utils::Factory class
 *
 * @author Frederic Py <fpy@mbari.org>
 * @ingroup utils
 */
#ifndef In_H_Factory
# error "tcc files cannot be included outside of their corresponding header" 
#else									

template<class P, class I, class C, class R, class O>
typename Factory<P, I, C, R, O>::producer const &Factory<P, I, C, R, O>::get
(typename Factory<P, I, C, R, O>::id_param id) const {
    typename Factory<P, I, C, R, O>::catalog_type::const_iterator 
    i = m_producers.find(id);
    if( m_producers.end()==i )
        throw UnknownFactoryType("No producer associated to this ID");
    return *(i->second); 
}

template<class P, class I, class C, class R, class O>
bool Factory<P, I, C, R, O>::exists
(typename Factory<P, I, C, R, O>::id_param id) const {
    return m_producers.end()!=m_producers.find(id);
}

template<class P, class I, class C, class R, class O>
void Factory<P, I, C, R, O>::add
(typename Factory<P, I, C, R, O>::producer const *prod) {
    typename Factory<P, I, C, R, O>::catalog_type::value_type
    to_ins(prod->getId(), prod);
    if( !m_producers.insert(to_ins).second ) {
# if 0 // I have a weird bug some time with that ... to be checked
        throw MultipleFactoryDecl("ID multiply used.");
# endif 
    }
}

template<class P, class I, class C, class R, class O>
void Factory<P, I, C, R, O>::remove
(typename Factory<P, I, C, R, O>::producer const *prod) {
    typename Factory<P, I, C, R, O>::catalog_type::iterator
    i = m_producers.find(prod->getId());
    if( m_producers.end()!=i && i->second==prod ) 
        m_producers.erase(i);
}

template<class P, class I, class C, class R, class O>
void Factory<P, I, C, R, O>::getIds(std::list<I> &ids) const {
    typename Factory<P, I, C, R, O>::catalog_type::const_iterator
    i = m_producers.begin();
    for( ; m_producers.end()!=i; ++i)
        ids.push_back(i->first);
}


#endif // In_H_Factory
