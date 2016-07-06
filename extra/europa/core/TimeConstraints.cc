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
#include "trex/europa/TimeConstraints.hh"
#include "trex/europa/Assembly.hh"

// include plasma header as system files in order to disable warnings
# define TREX_PP_SYSTEM_FILE <PLASMA/Domain.hh>
# include <trex/europa/bits/system_header.hh>

# include <trex/utils/chrono_helper.hh> 

using namespace TREX::europa;

namespace {

  bool date_to_europa(Assembly::pdate const &date,
		      EUROPA::edouble &seconds,
		      EUROPA::eint    &days,
		      EUROPA::eint    &years) {
    if( date.is_neg_infinity() ) {
      seconds = std::numeric_limits<EUROPA::edouble>::minus_infinity();
      days = std::numeric_limits<EUROPA::eint>::minus_infinity();
      years = std::numeric_limits<EUROPA::eint>::minus_infinity();
    } else if( date.is_pos_infinity() ) {
      seconds = std::numeric_limits<EUROPA::edouble>::infinity();
      days = std::numeric_limits<EUROPA::eint>::infinity();
      years = std::numeric_limits<EUROPA::eint>::infinity();
    } else if( date.is_special() ) {
      return false;
    } else {
      typedef TREX::utils::chrono_posix_convert< CHRONO::duration<EUROPA::edouble::basis_type> > convert;
      seconds = convert::to_chrono(date.time_of_day()).count();
      days = (int)date.date().day_of_year();
      years = (int)date.date().year();
    }
    return true;
  }

  Assembly::pdate europa_to_date(EUROPA::edouble seconds,
				 EUROPA::eint    days,
				 EUROPA::eint    years) {    
    typedef TREX::utils::chrono_posix_convert< CHRONO::duration<EUROPA::edouble::basis_type> > convert;
    CHRONO::duration<EUROPA::edouble::basis_type> time_of_day(EUROPA::cast_basis(seconds));

    Assembly::pdate::date_type jan_1st(EUROPA::cast_basis(years), 1, 1);
    return Assembly::pdate(jan_1st,
			   boost::posix_time::hours(24*(EUROPA::cast_basis(days)-1))+
			   convert::to_posix(time_of_day));
  }
	       

}

/*
 * class TREX::europa::TickFromDate
 */
 
// structors 

TickFromDate::TickFromDate(EUROPA::LabelStr  const &name, 
                           EUROPA::LabelStr const &propagatorName,
                           EUROPA::ConstraintEngineId const &cstrEngine, 
                           std::vector<EUROPA::ConstrainedVariableId> const &variables)
  :ReactorConstraint(name, propagatorName, cstrEngine, variables) {
  if( m_variables.size()!=TickFromDate::NARGS )
    throw EuropaException("Constraint "+name.toString()+" accept exactly 4 arguments");
  m_tick = &getCurrentDomain(m_variables[TickFromDate::TICK]);
  m_seconds = &getCurrentDomain(m_variables[TickFromDate::SECONDS]);
  m_days = &getCurrentDomain(m_variables[TickFromDate::DAYS]);
  m_years = &getCurrentDomain(m_variables[TickFromDate::YEARS]);
}

// manipulators
 
void TickFromDate::handleExecute() {
  Assembly &a = assembly();
  
  EUROPA::edouble t_lo, t_hi;
  Assembly::pdate d_lo(boost::posix_time::neg_infin), d_hi(boost::posix_time::pos_infin);
  
  m_tick->getBounds(t_lo, t_hi);
  
  
  // First do the tick to date
  if( t_lo>std::numeric_limits<EUROPA::eint>::minus_infinity() ) 
    d_lo = a.tick_to_date(t_lo);
  if( t_hi<std::numeric_limits<EUROPA::eint>::infinity() )
    d_hi = a.tick_to_date(t_hi);
  
  EUROPA::edouble s_lo, s_hi;
  EUROPA::eint    day_lo, day_hi, y_lo, y_hi;
  
  // convert my dates into (year,day_of_year,seconds_of_day)
  date_to_europa(d_lo, s_lo, day_lo, y_lo);
  date_to_europa(d_hi, s_hi, day_hi, y_hi);

  if( y_lo<=1400 )
    y_lo = 1401;
  if( y_hi>=10000 )
    y_hi = 9999;
    
  if( y_lo<y_hi ) {
    // this covers one or more years
    //  => relax day to be in the range of a year
    //  NOTE: could be improved by checking idf the upper bound is 366 or 365
    day_lo = 1;
    day_hi = 366;
  }
  if( day_lo < day_hi ) {
    // this covers multiople days
    //  => relax the sdeconds to cover a full day
    s_lo = 0.0;
    s_hi = 3600.0 * 24.0;
  }
  // Now do the intersection
  if( m_seconds->intersect(s_lo, s_hi)
      &&  m_seconds->isEmpty() )
    return;
  if( m_days->intersect(day_lo, day_hi)
      && m_days->isEmpty() )
    return;
  if( m_years->intersect(y_lo, y_hi)
      && m_years->isEmpty() )
    return;

  EUROPA::edouble fy_lo, fy_hi, fd_lo, fd_hi;

  // OK now convert the date to a tick
  m_years->getBounds(fy_lo, fy_hi);
  // At this point the day should already been bounded to [0 366] 
  m_days->getBounds(fd_lo, fd_hi);
  // At this point the seconds shold be already bounded to [0.0, 86400.0]
  m_seconds->getBounds(s_lo, s_hi);

  // TODO extract those from boost
  if( fy_lo > 1400 )
    t_lo = a.date_to_tick(europa_to_date(s_lo, fd_lo, fy_lo));
  if( fy_hi < 10000 )
    t_hi = a.date_to_tick(europa_to_date(s_hi, fd_hi, fy_hi));
  m_tick->intersect(t_lo, t_hi);
}
