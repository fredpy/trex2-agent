/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Frederic Py.
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
#ifndef H_trex_domain_token
# define H_trex_domain_token

# include "var.hh"
# include "int_domain.hh"

# include <boost/signals2/signal.hpp>

# include <functional>

namespace TREX {
  namespace transaction {
    
    
    class token;
    typedef SHARED_PTR<token const> token_id;
    typedef SHARED_PTR<token> token_ref;
    
    
    class token: public TREX::utils::ptree_convertible,
    public ENABLE_SHARED_FROM_THIS<token> {
    public:
      struct attr_comp
      :public std::binary_function<utils::symbol, utils::symbol, bool> {
        
        bool hidden(utils::symbol const &a) const;
        bool operator()(utils::symbol const &a, utils::symbol const &b) const;
      }; // token::attr_comp
      
      
    private:
      typedef std::map<utils::symbol, var, attr_comp> attr_set;
      
    public:
      typedef boost::signals2::signal<void (token const &,
                                            var const &)> update_sig;
      
      
      static utils::symbol const obs_tag;
      static utils::symbol const goal_tag;
      
      static token_ref obs(utils::symbol const &obj,
                           utils::symbol const &pred) {
        token_ref ret = MAKE_SHARED<token>(obj, pred);
        ret->pred_tag(obs_tag);
        return ret;
      }
      static token_ref goal(utils::symbol const &obj,
                            utils::symbol const &pred) {
        token_ref ret = MAKE_SHARED<token>(obj, pred);
        ret->pred_tag(goal_tag);
        return ret;
      }
      
      static utils::symbol const s_failed;
      static utils::symbol const s_undefined;
      
      update_sig &on_update() {
        return m_updated;
      }
      
      typedef attr_set::const_iterator attr_iterator;
      
      static utils::symbol const s_start;
      static utils::symbol const s_duration;
      static utils::symbol const s_end;
      
      static int_domain const s_date_full;
      static int_domain const s_duration_full;
      
      token(utils::symbol const &obj, utils::symbol const &pred);
      token(boost::property_tree::ptree::value_type &node);
      token(token const &other);
      ~token();
      
      token &operator= (token const &other);
      
      attr_iterator attr_begin(bool all=true) const;
      attr_iterator attr_end(bool all=true) const;
      
      token_id id() const {
        return shared_from_this();
      }
      
      utils::symbol const &object() const {
        return m_object;
      }
      utils::symbol const &predicate() const {
        return m_pred;
      }
      
      void list_attributes(std::list<utils::symbol> &attrs,
                           bool all = false) const;
      
      bool has_attribute(utils::symbol const &name) const;
      var const &attribute(utils::symbol const &name) const;
      var const &operator[](utils::symbol const &name) const {
        return attribute(name);
      }
      template<class Ty>
      Ty const &domain(utils::symbol const &name) const;
      
      int_domain const &start() const {
        return m_start->second.typed_domain<int_domain>();
      }
      int_domain const &duration() const {
        return m_duration->second.typed_domain<int_domain>();
      }
      int_domain const &end() const {
        return m_end->second.typed_domain<int_domain>();
      }
      
      bool is_temporal(attr_iterator const &i) const;
      bool is_full(attr_iterator const &i) const;

      void restrict_attribute(var const &v, ERROR_CODE &ec);
      void restrict_attribute(var const &v);
      void restrict_attribute(utils::symbol const &name,
                              abstract_domain const &d) {
        return restrict_attribute(var(name, d));
      }
      /** @brief "Merge" 2 tokens
       *
       * @tparam[in] other Another token
       *
       * This method constrain current instance with @p other. 
       * This constraint is in the sense that it changes this instance 
       * as the intersecetion between the current instance and @p other
       *
       * @pre consistent_with(other)
       * @throw SYSTEM_ERROR @p other was not consistent with this instance
       *  which  means that either:
       *   @li the 2 tokens are not on the same object
       *   @li the 2 tokens are not the seme predicate
       *   @li one attribute intersection domain between other and this 
       *       token is empty
       *
       * @sa consistent_with(token const &)
       * @post this token attributes have been restricted with 
       *       the attributes of other
       *
       * @note At the current stage the merge operation do not maintain 
       *   the tokens it is merged to. Which mean that if @p other 
       *   is updated later on these updates will not propagate to this 
       *   instance unless this method is called again explicitely.
       */
      void merge_with(token const &other, ERROR_CODE &ec);
      void merge_with(token const &other);
      
      void restrict_time(int_domain const &s,
                         int_domain const &d,
                         int_domain const &e,
                         ERROR_CODE &ec);
      void restrict_time(int_domain const &s,
                         int_domain const &d,
                         int_domain const &e);
      void restrict_start(int_domain const &d, ERROR_CODE &ec) {
        restrict_time(d, s_duration_full, s_date_full, ec);
      }
      void restrict_start(int_domain const &d) {
        restrict_time(d, s_duration_full, s_date_full);
      }
      void restrict_duration(int_domain const &d, ERROR_CODE &ec) {
        restrict_time(s_date_full, d, s_date_full, ec);
      }
      void restrict_duration(int_domain const &d) {
        restrict_time(s_date_full, d, s_date_full);
      }
      void restrict_end(int_domain const &d, ERROR_CODE &ec) {
        restrict_time(s_date_full, s_duration_full, d, ec);
      }
      void restrict_end(int_domain const &d) {
        restrict_time(s_date_full, s_duration_full, d);
      }
      
      void pred_tag(utils::symbol const &p) {
        m_tag = p;
      }
      utils::symbol const &pred_tag() const {
        return m_tag;
      }
      
      boost::property_tree::ptree as_tree() const {
        return as_tree(true);
      }
      boost::property_tree::ptree as_tree(bool full) const;
      
      bool consistent_with(token const &other) const;
      
      typedef utils::xml_factory<token, token_ref> factory;
      
      
      class declare :public factory::factory_type::producer {
      public:
        typedef factory::factory_type::id_param id_param;
        
        declare(id_param id);
        
        result_type produce(argument_type arg) const;
      }; // class TREX::transaction::token::declare
      
      bool starts_after(int_domain::base_type date,
                        int_domain::base_type delay=0);
      bool starts_before(int_domain::bound const &date) const {
        return start().lower_bound()<date;
      }
      bool ends_after(int_domain::base_type date) const {
        return end().lower_bound()>date;
      }
      
      
    private:
      update_sig    m_updated;
      utils::symbol m_object, m_pred, m_tag;
      attr_set      m_attrs;
      
      attr_set::iterator m_start, m_duration, m_end;
      
      void init_time();
      attr_set::iterator constrain(var const &v, bool &updated, ERROR_CODE &ec);
      
      token() DELETED;
    }; // TREX::transaction::token
    
    template<class Ty>
    Ty const &token::domain(utils::symbol const &name) const {
      return attribute(name).typed_domain<Ty>();
    }

    std::ostream &operator<<(std::ostream &out, token const &tok);
    
  }
}

#endif // H_trex_domain_token