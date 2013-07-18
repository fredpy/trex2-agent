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
#ifndef H_TREX_witre_TREXLogWidget
# define H_TREX_witre_TREXLogWidget

# include "WitreServer.hh"

# include <Wt/WContainerWidget>
# include <Wt/WPopupMenu>
# include <Wt/WText>
// # include <Wt/WCssStyleSheet>

namespace TREX {
  namespace witre {
    
    class TREXLogWidget :public Wt::WContainerWidget {
    public:
      TREXLogWidget(WitreServer &server, 
                    Wt::WContainerWidget *parent =0);
      virtual ~TREXLogWidget();
      
      void new_msg(std::string type, boost::optional<long long> date,
                   std::string src, std::string content);
      
    private:
      void show_hide(Wt::WPopupMenuItem *item);
      
      class Entry :public Wt::WText {
      public:
        Entry(boost::optional<long long> date, 
              std::string src, std::string content,
              TREXLogWidget *parent = NULL);
        ~Entry() {}
      };
      
      void add_msg(std::string type, boost::optional<long long> date,
                   std::string src, std::string content);
      
      
      std::map<std::string, Wt::WCssTemplateRule *> m_types;
      Wt::WPopupMenu m_type_select;
    }; // TREX::witre::TREXLogWidget
     
    
    
  } // TREX::witre
} // TREX

#endif // H_TREX_witre_TREXLogWidget
