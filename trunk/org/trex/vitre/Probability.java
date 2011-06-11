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
package org.trex.vitre;

import java.io.*;

public class Probability {

	//This method creates a file name Probability.log in the TREX log directory containing
	//all the calculated probabilities for each tick in which signalTracker is active
	//notice that it looks for prevP as currentP is still unbound in TREX.log
	//it also marks the points in which gulpers have been fired if fired = true
	public void findProbs (String folder, boolean fired){
		try {	
			BufferedReader Logfile;
			PrintWriter outputFile ;
			String line ="";
			Logfile = new BufferedReader (new FileReader (folder+"/"+"TREX.log"));
			outputFile = new PrintWriter (folder+"/"+"Probability.log");
			while (!(line==null)){
				line = Logfile.readLine();
				if (!(line==null) &&line.contains("prevP")){
					String proba = line.split("prevP")[1];
					String tick = (line.split("]")[1]);
					outputFile.println(tick+"] "+proba.substring(1,5));
				}
				if (fired && !(line==null) &&line.contains("=firing")){
					outputFile.println(line);
				}
			}
			outputFile.close();
		}
		catch (IOException e){
			System.out.println(e.toString());
		}

	}
	public static void main(String[] args) {
		// TODO Auto-generated method stub
		Probability p = new Probability();
		String folder;
		boolean gulper;
		if (args.length==0) folder="latest";
		else folder=args[0];
		if (args.length>1) gulper=false;
		else gulper = true;
		p.findProbs("../../log/" + folder,gulper);

	}
}
