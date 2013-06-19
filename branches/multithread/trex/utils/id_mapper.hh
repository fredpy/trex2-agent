/*********************************************************************
 * Software License Agreement (BSD License)
 * 
 *  Copyright (c) 2011, MBARI.
 *  All rights reserved.
 * 
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 * 
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the TREX Project nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 * 
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef H_id_mapper 
# define H_id_mapper

# include <functional>
# include <list>

# include <boost/call_traits.hpp>
# include <boost/function_types/result_type.hpp>
# include <boost/tuple/tuple.hpp>

namespace TREX {
  namespace utils {

    /** @brief list_set sorting functor
     *
     * @tparam IdTraits A traits for identifier extraction
     * @tparam Cmp      A comparison functor
     *
     * This is a functor used by the template list_set class to
     * know how to compare and sort its elements
     *
     * @author Frderic Py <fpy@mbari.org>
     * @ingroup utils
     * @relates class list_set
     */
    template< class IdTraits, class Cmp=std::less<typename IdTraits::id_type> >
    struct id_cmp {
      Cmp m_cmp;
    public:
      typedef typename IdTraits::base_type base_type;
      typedef typename IdTraits::id_type  id_type;

      /** @brief Constructor
       *
       * @param[in] cmp A comparison functor
       */
      id_cmp(Cmp const &cmp)
	:m_cmp(cmp) {}
      
      /** @brief Element comparison
       *
       * @param[in] a An alement
       * @param[in] b an element
       *
       * Compare @p a and @p b based on their idenifiers
       *
       * @retval true if the idenitifer of @p a is before the identifier of @p b
       * @retval false otherwise
       */
      bool operator()(base_type const &a, base_type const &b) const {
	return m_cmp(IdTraits::get_id(a), IdTraits::get_id(b));
      }
      /** @brief Identifer to element comparison
       *
       * @param[in] a An identifier
       * @param[in] b an element
       *
       * Compare @p a and  the identifier of @p b 
       *
       * @retval true if @p a is before the identifier of @p b
       * @retval false otherwise
       */
      bool operator()(id_type const &a, base_type const &b) const {
	return m_cmp(a, IdTraits::get_id(b));
      }
      /** @brief Element to identifier comparison
       *
       * @param[in] a An element
       * @param[in] b An idenitfier
       *
       * Compare @p a and  the identifier of @p b 
       *
       * @retval true if the identifier of @p a is before @p b
       * @retval false otherwise
       */
      bool operator()(base_type const &a, id_type const &b) const {
	return m_cmp(IdTraits::get_id(a), b);
      }
    }; // TREX::utils::id_cmp<>

    /** @brief Identifier extraction for pair based element
     *
     * @tparam IdTraits An idenitifer extraction traits for the first element
     * @tparam Ty       The type of the second element
     *
     * A traits used to generate a list_set equivalent to a std::map
     *
     * @author Frderic Py <fpy@mbari.org>
     * @ingroup utils
     * @relates class list_set
     */
    template<class IdTraits, class Ty>
    struct map_id_traits {
      typedef std::pair<typename IdTraits::base_type, Ty>  base_type;
      typedef typename IdTraits::id_type                   id_type;
      
      /** @brief Extract identifier
       *
       * @param p An element
       *
       * @return the identifier associated to @p p
       */
      static id_type get_id(base_type const &p) {
	return IdTraits::get_id(p.first);
      }
    }; // TREX::utils::map_id_traits<>

    /** @brief Identifier extraction for pointer based element
     *
     * @tparam IdTraits An idenitifer extraction traits for the first element
     * @tparam Ptr      A pointer like type 
     *
     * A traits used to generate a list_set containing pointer / smart pointer
     * elements 
     *
     * @author Frderic Py <fpy@mbari.org>
     * @ingroup utils
     * @relates class list_set
     */
    template<class IdTraits, class Ptr=typename IdTraits::base_type *>
    struct pointer_id_traits {
      typedef Ptr                         base_type;
      typedef typename boost::call_traits<base_type>::param_type argument_type;
      typedef typename IdTraits::id_type  id_type;

      /** @brief Extract identifier
       *
       * @param ptr An element
       *
       * @return the identifier associated to @p ptr
       */
      static id_type get_id(argument_type ptr) {
	return IdTraits::get_id(*ptr);
      }	
    }; // TREX::utils::pointer_id_traits<>
    
    
      

    /** @brief Embedded key comparison helper
     *
     * @tparam IdTraits key extraction traits
     * @tparam Cmp      key comparaison functor
     *
     * This class is a generalize implementation helper for sorted associative 
     * containers. It provide the basic methods to find an object in such
     * container. The generalization is made using : 
     * @li @p IdTraits to identify how to extract the key from an object
     * @li @p Cmp      to compare the keys from one to another
     *
     * While @p Cmp is a classical C++ sorting functor (@c std::less is
     * the default template value), @p IdTraits is the addition that allows
     * to generalize sorted associative container.
     *
     * An @p IdTraits is a simpel structure that needs to provide the following
     * static attributes :
     * @li @c id_type A typedef that defines the type of the key used
     * @li @c base_type The type of the input argument from where the key
     *                  can be extracted
     * @li @c id_type get_id(base_type) the method of extraction of the key
     *
     * For example the IdTraits fro a std::map can be defined as follow:
     * @code
     * template<typename Key, typename Val>
     * struct map_id_traits {
     *   typedef std::pair<Key const, Val> base_type;
     *   typedef Key                       id_type;
     *
     *   static id_type const &get_id(base_type const &v) {
     *     return v.first;
     *   }
     * };
     * @endcode
     * 
     * The extractor for @c std::set class is even simpler as it maps
     * the object to itself:
     * @code
     * template<typename Ty>
     * struct set_id_traits {
     *   typedef Ty const  base_type;
     *   typedef base_type id_type;
     *
     *   static id_type const &get_id(base_type const &v) {
     *     return v;
     *   }
     * };
     * @endcode
     * 
     * @ingroup utils
     * @author Frederic Py <fpy@mbari.org>
     * @sa list_set
     */ 
    template<class IdTraits, class Cmp=std::less<typename IdTraits::id_type> >
    class id_mapper {
      /** @brief Key comparrison functor
       *
       * The functor used by this instance to sort the elements using their key
       */
      id_cmp<IdTraits, Cmp> m_cmp;

    public:
      typedef typename IdTraits::base_type                     base_type;
      /** @brief Type of the key
       *
       * This typedef extracts the type of the key from @p IdTraits
       */
      typedef typename IdTraits::id_type                       id_type;
      /** @brief Reference type of the key
       *
       * This typedef infers the best way to pass a key as an argeument
       * to the methods imnplemented here.
       */
      typedef typename boost::call_traits<id_type>::param_type reference;
      
      /** brief Constructor
       *
       * @param[in] A comparison functor
       *
       * Crate a new insatnce that will uses @p cmp to compare the different
       * keys
       */
      id_mapper(Cmp const &cmp = Cmp())
	:m_cmp(cmp) {}
      /** @brief Destructor */
      ~id_mapper() {}


      /** @brief iterator comparison to a key
       *
       * @tparam Iter the type of iterator
       * @param[in] x   An iterator
       * @param[in] id  A key value
       *
       * Check if the value referenced by @p x is before the key @p id
       *
       * @pre @p x points to a valid element
       *
       * @retval true if * @p x is before @p id
       * @retval false otherwise
       *
       * @sa after(Iter, reference) const
       */
      template<class Iter>
      bool before(Iter x, reference id) const {
	return m_cmp(*x, id);
      }
      /** @brief iterator comaparison to a key
       *
       * @tparam Iter the type of iterator
       * @param[in] x   An iterator
       * @param[in] id  A key value
       *
       * Check if the value referenced by @p x is after the key @p id
       *
       * @pre @p x points to a valid element
       *
       * @retval true if * @p x is after @p id
       * @retval false otherwise
       *
       * @sa before(Iter, reference) const
       */
      template<class Iter>
      bool after(Iter x, reference id) const {
	return m_cmp(id, *x);
      }

      /** @brief Lower bound of a key
       *
       * @tparam Iter an iterator type
       * @param[in] from starting point of the search
       * @param[in] to   ending point of the search
       * @param[in] id   A key
       *
       * Finds the first iterator in [@p from, @p to ) interval
       * whose key is not before @p id
       *
       * @pre [@p from, @p to) is a valid iterator interval
       * 
       * @return The first iterator for which the key is not before
       *         @p id or @p to if such iterator does not exist
       *
       * @sa before(Iter, reference) const
       * @sa upper_bound(Iter, Iter, reference) const
       */
      template<class Iter>
      Iter lower_bound(Iter from, Iter to, reference id) const {
	for( ; to!=from && before(from, id); ++from);
	
	return from;
      }

     /** @brief Upper bound of a key
       *
       * @tparam Iter an iterator type
       * @param[in] from starting point of the search
       * @param[in] to   ending point of the search
       * @param[in] id   A key
       *
       * Finds the first iterator in [@p from, @p to ) interval
       * whose key is after @p id
       *
       * @pre [@p from, @p to) is a valid iterator interval
       * 
       * @return The first iterator for which the key is after
       *         @p id or @p to if such iterator does not exist
       *
       * @sa after(Iter, reference) const
       * @sa lower_bound(Iter, Iter, reference) const
       */
      template<class Iter>
      Iter upper_bound(Iter from , Iter to, reference id) const {
	for( ; to!=from && !after(from, id); ++from);
	
	return from;
      }
    }; // TREX::utils::id_mapper

    /** @brief List based generalized set
     *
     * @tparam IdTraits A traits that helps element key extraction
     * @tparam Cmp      A comparison functor
     *
     * This class is a simple generalized implementation of a set/map
     * using a @c std::list as the internal container.
     *
     * It relies on the id_mapper class to locate elements
     *
     * @note This implementation is a simple assovciative container. It
     *       implies that it ensure that there's one and only one element
     *       with a given key
     *
     * @sa class id_mapper
     * @ingroup utils
     * @author Frederic Py <fpy@mbari.org>
     */
    template< class IdTraits, class Cmp=std::less<typename IdTraits::id_type> >
    class list_set {
    public:
      /** @brief Elements type
       *
       * The type of the element stored.
       */
      typedef typename IdTraits::base_type value_type;
      /** @brief Key type
       *
       * The type of the key used to dsort the element.
       */
      typedef typename IdTraits::id_type   key_type;

    private: 
      /** @brief Subjacent container type
       *
       * The type used internally to stroe the different element
       * of this container.
       *
       * @note The choice of std::list was enforced by the interresting properties
       * of the iterators in this class. Mostly a reactor is not invalidated
       * by the modification of this list except if it is removed from it.
       * This peroperty os also true for a @c std::set but the implemetation
       * of a generic search based on the key would have been much more difficult
       */
      typedef std::list<value_type> container_type;

      container_type           m_list;   //!< iternal elements container
      id_mapper<IdTraits, Cmp> m_mapper; //!< Key mapper
    public:
      typedef typename container_type::size_type       size_type;
      typedef typename container_type::difference_type difference_type;
      typedef typename container_type::iterator        iterator; //!< elements iterator 
      typedef typename container_type::const_iterator const_iterator; //!< elements const iterator

      /** @brief Default constructor
       *
       * Create an empty set
       */
      list_set() {}
      /** @brief copy constructor
       *
       * @param[in] other Another instance
       *
       * Create a copy of @p other
       */
      list_set(list_set const &other) 
	:m_list(other.m_list) {}
      /** @brief Constructor
       *
       * @tparam Iter an iterator
       * @param from start iterator
       * @param to   end iterator
       *
       * Create A setby inserting all the elements in the interval
       * [@p from, @p to)
       * @note If multiple elements have the same key only the first
       * found will be insserted
       */
      template<class Iter>
      list_set(Iter from, Iter to) {
	for( ; to!=from; ++from) 
	  insert(*from);
      }
      /** @brief Destructor */
      ~list_set() {}

      /** @brief Check if empty
       *
       * @retval true if current instance has no element
       * @retval false otherwise
       */
      bool empty() const {
	return m_list.empty();
      }

      /** @brief Number of element
       *
       * @return The number of element stored
       */
      size_type size() const {
	return m_list.size();
      }

      /** @brief Beginning iterator
       *
       * @return An iterator pointing to the beginning of
       * the list
       *
       * @sa begin() const 
       * @sa end()
       */
      iterator begin() {
	return m_list.begin();
      }
      /** @brief End iterator
       *
       * @return An iterator pointing to the end of
       * the list
       *
       * @sa begin()  
       * @sa end() const
       */
      iterator end() {
	return m_list.end();
      }
      /** @brief Beginning iterator
       *
       * @return An iterator pointing to the beginning of
       * the list
       *
       * @sa begin()
       * @sa end() const
       */
      const_iterator begin() const {
	return m_list.begin();
      }
      /** @brief End iterator
       *
       * @return An iterator pointing to the end of
       * the list
       *
       * @sa begin() const 
       * @sa end() 
       */
      const_iterator end() const {
	return m_list.end();
      }

      /** @brief Lower bound
       *
       * @param[in] k A key
       *
       * Finds the first element whose key is not before @p k
       *
       * @return An iterator refering to the first element that is not
       *         before @p k or end() if such element does not exist
       *
       * @sa lower_bound(value_type const &)         
       * @sa upper_bound(key_type const &)
       * @sa id_mapper::lower_bound(Iter,Iter,id_mapper::reference) const
       *
       * @{
       */
      iterator lower_bound(key_type const &k) {
	return m_mapper.lower_bound(begin(), end(), k);
      }
      const_iterator lower_bound(key_type const &k) const {
	return m_mapper.lower_bound(begin(), end(), k);
      }
      /** @} */

      /** @brief Lower bound
       *
       * @param[in] v A value
       *
       * Finds the first element whose key is not before the key of @p v
       *
       * @return An iterator refering to the first element that is not
       *         before @p v or end() if such element does not exist
       *
       * @sa lower_bound(key_type const &)         
       * @sa upper_bound(value_type const &)
       * @sa id_mapper::lower_bound(Iter,Iter,id_mapper::reference) const
       *
       * @{
       */
      iterator lower_bound(value_type const &v) {
	return lower_bound(IdTraits::get_id(v));
      }
      const_iterator lower_bound(value_type const &v) const { 
	return lower_bound(IdTraits::get_id(v));
      }
      /** @} */

      /** @brief Upper bound
       *
       * @param[in] k A key
       *
       * Finds the first element whose key is after @p k
       *
       * @return An iterator refering to the first element that is 
       *         after @p k or end() if such element does not exist
       *
       * @sa upper_bound(value_type const &)          
       * @sa lower_bound(key_type const &)
       * @sa id_mapper::upper_bound(Iter,Iter,id_mapper::reference) const
       *
       * @{
       */
      iterator upper_bound(key_type const &k) {
	return m_mapper.upper_bound(begin(), end(), k);
      }
      const_iterator upper_bound(key_type const &k) const {
	return m_mapper.upper_bound(begin(), end(), k);
      }
      /** @} */


      /** @brief Upper bound
       *
       * @param[in] v A value
       *
       * Finds the first element whose key is after the key of @p v
       *
       * @return An iterator refering to the first element that is 
       *         after @p v or end() if such element does not exist
       *
       * @sa upper_bound(key_type const &)          
       * @sa lower_bound(value_type const &)
       * @sa id_mapper::upper_bound(Iter,Iter,id_mapper::reference) const
       *
       * @{
       */
      iterator upper_bound(value_type const &v) {
	return upper_bound(IdTraits::get_id(v));
      }
      const_iterator upper_bound(value_type const &v) const {
	return upper_bound(IdTraits::get_id(v));
      }
      /** @} */
      
      /** @brief Equal range
       *
       * @param[in] k A key
       *
       * Finds the iterators that correspond to all the elements
       * of the set whose key is @p k
       *
       * @return a pair defining the iterator interval of all the elements
       * whose key is @p k
       *
       * @note As we have a unique associative container ther should be at most
       * one element in this interval
       *
       * @sa lower_bound(key_type const &)
       * @sa upper_bound(key_type const &)
       * @sa equal_range(value_type const &v)
       *
       * @{
       */
      std::pair<iterator, iterator> equal_range(key_type const &k) {
	iterator end_p = end(),
	  pos = m_mapper.lower_bound(begin(), end_p, k);
	return std::make_pair(pos, m_mapper.upper_bound(pos, end_p, k));
      }	
      std::pair<const_iterator, const_iterator> equal_range(key_type const &k) const {
	const_iterator end_p = end(),
	  pos = m_mapper.lower_bound(begin(), end_p, k);
	return std::make_pair(pos, m_mapper.upper_bound(pos, end_p, k));
      }
      /** @} */

      
      /** @brief Equal range
       *
       * @param[in] v An element value
       *
       * Finds the iterators that correspond to all the elements
       * of the set whose key is the same as the key of @p v
       *
       * @return a pair defining the iterator interval of all the elements
       * whose key is @p v
       *
       * @note As we have a unique associative container ther should be at most
       * one element in this interval
       *
       * @sa lower_bound(value_type const &)
       * @sa upper_bound(value_type const &)
       * @sa equal_range(key_type const &)
       *
       * @{
       */
      std::pair<iterator, iterator> equal_range(value_type const &v) {
	return equal_range(IdTraits::get_id(v));
      }	
      std::pair<const_iterator, const_iterator> equal_range(value_type const &v) const {
	return equal_range(IdTraits::get_id(v));
      }
      /** @} */

      /** @brief Element insertion
       *
       * @param[in] v An element value
       *
       * This method Find the position of @p v in the set and insert
       * it if there is no element with the same key as @p v
       *
       * @return A pair with the @c first element being the iterator pointing
       * to @p v position in thins set and @c second a @c bool which is @c true
       * if @p v was inserted and @p false if an element with the same key as @p
       * already exist
       *
       * In the case @c second is @c false it is up to the coder to decide whether
       * he wants to overwrite the former value with @p v or not
       */
      std::pair<iterator, bool> insert(value_type const &v) {
	iterator from, to;
	bool inserted = false;

	boost::tie(from, to) = equal_range(IdTraits::get_id(v));
	if( from==to ) {
	  from = m_list.insert(from, v);
	  inserted = true;
	}
	return std::make_pair(from, inserted);
      }
      
      /** @brief Find element
       * 
       * @param[in] k A key
       *
       * Finds the elemnt whose key is @p k
       *
       * @return An iterator pointing to the first element whose key is @p k
       *         or end() if such element does not exist
       *
       * @sa equal_range(key_type const &)
       * @sa find(value_type const &)
       *
       * @{
       */
      iterator find(key_type const &k) {
	iterator from, to;
	
	boost::tie(from, to) = equal_range(k);
	if( from==to ) 
	  return end();
	return from;
      }
      const_iterator find(key_type const &k) const {
	const_iterator from, to;
	
	boost::tie(from, to) = equal_range(k);
	if( from==to ) 
	  return end();
	return from;
      }
      /** @} */
      
      /** @brief Find element
       * 
       * @param[in] v An element value
       *
       * Finds the element whose key is the same as @p v
       *
       * @return An iterator pointing to the first element whose key is the
       *         same as @p v or end() if such element does not exist
       *
       * @sa equal_range(value_type const &)
       * @sa find(key_type const &)
       *
       * @{
       */
      iterator find(value_type const &v) {
	return find(IdTraits::get_id(v));
      }
      const_iterator find(value_type const &v) const {
	return find(IdTraits::get_id(v));
      }
      /** @} */

      /** @brief Remove an element
       *
       * @param[in] pos element to remove
       *
       * @pre @p pos is a valid iterator for this instance
       *
       * @return the iterator after @p pos 
       */
      iterator erase(iterator pos) {
	return m_list.erase(pos);
      }
      /** @brief Remove elements
       *
       * @param[in] from first element to remove
       * @param[to] to   first element to not remove after @p from
       *
       * @pre [@p pos, @p to) is a valid iterator interval for this instance
       * 
       * @return @p to
       */
      iterator erase(iterator from, iterator to) {
	return m_list.erase(from, to);
      }
      /** @brief Remove an element
       *
       * @param[in] k A key 
       *
       * Remove the element whose key is @p k if it exist
       *
       * @treturn the iterator after the expected position
       * of element with key @p k
       */
      iterator erase(key_type const &k) {
	iterator from, to;
	
	boost::tie(from ,to) = equal_range(k);
	return erase(from, to);
      }

      /** @brief clear container
       *
       * Remove all the elements stroed in this instance
       */
      void clear() {
	m_list.clear();
      }

      value_type const &front() const {
	return m_list.front();
      }
      value_type &front() {
	return m_list.front();
      }
      void pop_front() {
	m_list.pop_front();
      }
      
    }; // TREX::utils::list_set
    
    

  } // TREX::utils
} // TREX

#endif // H_id_mapper
