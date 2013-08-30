#!/bin/bash
#####################################################################
# Software License Agreement (BSD License)
# 
#  Copyright (c) 2011, MBARI.
#  All rights reserved.
# 
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
# 
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the TREX Project nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
# 
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
#####################################################################

if [ $# -lt 2 ]; then 
    echo "Usage $0 host json-file"
    exit 1
else
    if [ -r $2 ]; then 
	url="http://$1/rest"    
	echo "Check for current tick"
	curl -X GET $url/tick 
	success=$?
	if [ $success -eq 0 ]; then 
	    echo "Posting the goal in $2 to $1"
	    curl -X POST --header 'Content-Type:application/json' -d @$2 $url/goal
	    sucess=$?
	    echo ""
	    exit $success
	else
	    echo "Failed to reach host $url"
	    echo "Usage $0 host goalfile"
	    exit $success
	fi
    else
       echo "cannot read file $2"
       echo "Usage $0 host goalfile"
       exit $success
    fi
fi 
