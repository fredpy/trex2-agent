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
#ifndef H_utils_ptree_utils
# define H_utils_ptree_utils

# include <boost/property_tree/ptree.hpp>

namespace TREX {
  namespace utils {
    
    void flatten_xml_attrs(boost::property_tree::ptree &p);
    void flatten_json_arrays(boost::property_tree::ptree &p);

    void read_xml(std::istream &in, boost::property_tree::ptree &p);
    void write_xml(std::ostream &out, boost::property_tree::ptree p,
                   bool header=false);

    void read_json(std::istream &in, boost::property_tree::ptree &p);
    void write_json(std::ostream &out, boost::property_tree::ptree p,
                      bool fancy=true);
    
    class ptree_convertible {
    public:      
      virtual boost::property_tree::ptree as_tree() const =0;
      
      std::ostream &to_xml(std::ostream &out) const;
      std::ostream &to_json(std::ostream &out) const;
    protected:
      ptree_convertible() {}
      virtual ~ptree_convertible() {}
    };
    
  } // TREX::utils
} // TREX

#endif // H_utils_json_utils
