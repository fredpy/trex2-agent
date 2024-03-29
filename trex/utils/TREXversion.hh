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
#ifndef H_TREXversion
# define H_TREXversion

# include <string> 

namespace TREX {
    
  /** @brief Core libraries version information
   * 
   * This simple class lows to access to information about the current TREX 
   * version of TREX core libararies
   * 
   * @ingroup utils
   * @author Frederic Py <fpy@mbari.org>
   */
  class version {
  public:
    version() {}
    ~version() {}
    /** @brief version number
     * 
     * Indicates the version number of this library. This version number follows 
     * the same nomenclature as th boost versionning and is guaranteed to 
     * increase as we have new versions. 
     * 
     * @return 100*(100*major()+minor())+release()
     * @sa major()
     * @sa minor()
     * @sa release()
     * @sa is_release_candidate()
     * @sa str()
     *
     * @note For backward compatibility reasons -- along with the fact 
     * that sorting versions numbers encoded as int with RC is not trivial
     * -- this number @e do @e not include the rc number. RC number should 
     * be tested aside knowing that 0.4.0-rc1 < 0.4.0-rc2 < ... < 0.4.0
     */
    static unsigned long number();

    /** @brief Major version number
     * @return the major version number
     */
    static unsigned short major_number();
    /** @brief Minor version number
     * @return the minor version number
     */
    static unsigned short minor_number();
    /** @brief Patch version number
     * @return the patch version number
     */
    static unsigned short release_number();

    /** @brief Check if reelase candidate
     *
     * Checks if this version is a release cnadidate or not
     * @sa rc_number()
     */
    [[deprecated]] static bool is_release_candidate() {
      return false;
    }
    /** @brief release cancdidate version 
     *
     * @return The release candidate version number for this version or 0
     *   if this version is not an RC
     * @sa is_release_candidate()
     */
    [[deprecated]] static unsigned rc_number() {
      return 0;
    }

    /** @brief Human readable version
     * 
     * Indicates the version number as a human readable string
     * the string is formatted the usual way :
     * @<major@>.@<minor@>.@<patch@>[-rc@<rc@>]
     *
     * @return The string value for this version
     * @sa major()
     * @sa minor()
     * @sa release()
     * @sa rc_number()
     * @sa number()
     */
    static std::string str();

    static std::string full_str();
    
    static bool git_info();
    static std::string git_branch();
    static std::string git_revision();
    
  }; // TREX::version
  
} // TREX

#endif // H_TREXversion
