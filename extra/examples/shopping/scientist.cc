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
#include <iostream>
#include <ctime>

#include <trex/domain/EnumDomain.hh>
#include <trex/domain/StringDomain.hh>

#include "scientist.hh"

using namespace TREX::utils;
using namespace TREX::transaction;
using namespace TREX::Scientist;

static int Horizon = 200;

Symbol const Scientist::auv("auv");
Symbol const Scientist::Sample("Sample");

Symbol const Scientist::Objective("objective");
std::string const Scientist::Vent1("Vent1");
std::string const Scientist::Vent2("Vent2");

Scientist::Scientist(TeleoReactor::xml_arg_type arg)
  :TeleoReactor(arg, false)
{

}

Scientist::~Scientist() {}

void Scientist::handleInit() {
    use(auv, true, false);
    srand( time(NULL) );
    //number = rand() % (Horizon-50) + 1;
}

bool Scientist::synchronize() {

    if(getCurrentTick()==100)
    {
        Goal goal(auv,Sample);
        transaction::Variable temp(Objective, TREX::transaction::StringDomain(Vent1));
        goal.restrictAttribute(temp);
        goal.restrictEnd(TREX::transaction::IntegerDomain(getCurrentTick()+1, Horizon));
        postGoal(goal);
    }
    return true;
}
