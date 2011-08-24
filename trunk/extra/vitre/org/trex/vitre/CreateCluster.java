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

public class CreateCluster {
	private int clusters=0, activePoints=0, stability=0, distribution=-1;
	private float max=0, min=-1, mean=0;
	private String file=null;
	public void asking () throws IOException {
		//Asking for the cluster parameters
		boolean error=true;
		BufferedReader read = new BufferedReader (new InputStreamReader (System.in));
		while (error){
			try {
				if (file==null || file == ""){
				System.out.println("Introduce the name of the output file");
				file = read.readLine();
				}
				while (clusters <=0){
					System.out.println("Introduce the total number of clusters");
					clusters = Integer.parseInt(read.readLine());
				}	
				while (activePoints <=0 || activePoints>clusters){
					System.out.println("Introduce the number of different probability points to be generated:");
					System.out.println('\t'+"Probability of remaining clusters will be interpolated, so more points imply sharpener transitions" +
							" among clusters");
					activePoints = Integer.parseInt(read.readLine());	
				}
				while (stability <=0 || stability>clusters/(activePoints-1)){
					System.out.println("Introduce the number of contiguous points that will have the same probability " +
							"(points per step):");
					System.out.println('\t'+"A 1 means linear interpolation, " +
							"the biggest the number the less the steps, " +
							"so the sharpener the transition between one step and the following");
					stability = Integer.parseInt(read.readLine());	
				}
				while (max <=0 || max>1){
					System.out.println("Introduce the maximum value of the probability (0 to 1):");
					max = Float.parseFloat(read.readLine());
					if (max==1) max= 0.9999F;
				}
				while (min <0 || min>=max){
					System.out.println("Introduce the minimum value of the probability (0 to 1):");
					min = Float.parseFloat(read.readLine());
				}
				while (mean <=0 || mean>max || mean < min){
					System.out.println("Introduce the mean value of the probability (0 to 1):");
					mean = Float.parseFloat(read.readLine());
				}
				while (distribution <0 || distribution >4){			
					System.out.println("Introduce the high probability points distribution profile:");
					System.out.println('\t'+"0: Random distribution");
					System.out.println('\t'+"1: Higher at the beginning");
					System.out.println('\t'+"2: Higher at the end");
					System.out.println('\t'+"3: Higher at the middle");
					System.out.println('\t'+"4: Higher at the extremes");
					distribution = Integer.parseInt(read.readLine());
				}
				error=false;
			}
			catch (NumberFormatException e){
				System.out.println("Error introducing data, should be a number: "+e.toString());
				error = true;
			}
		}
	}
	
	public float [] createClusters (){
		float values [] = new float [clusters];
		for (int i = 0; i<clusters;i++) values[i]=-1;
		//we create an array to put the activePoints
		float active [] = new float [activePoints];
		int counter;
		//We choose random values for the activePoints, but 2 first are max and min
		active[0]= min;
		active[1] = max;
		for (counter = 2; counter <activePoints; counter++){
			active[counter]= (float) (Math.random()*(max-min)+min);
		}
		float currentMean = calculateMean (active);
		//while the currentMean is not the required one plus minus 0.0001
		while (Math.abs(mean-currentMean)>0.0001){
			//We select one random point and change its probability in 0.001
			int point = (int) (Math.random()*(activePoints-2)+2);
			//if the required mean is bigger than the current we increase the value of the point
			if (mean>currentMean){
				active[point]= active[point]+0.0001F;
				//if the result is bigger than max we set this point to the max
				if (active[point]> max) active[point]= max;
			}
			else {
				active[point]= active[point]-0.001F;
				//if the result is bigger than max we set this point to the max
				if (active[point]< min || active[point]<0.0001) active[point]= min;
			}
			currentMean = calculateMean (active);
			//To test the convergence of the method
	//		for (int i=0; i<activePoints; i++) System.out.print(active[i]+",");
	//		System.out.println(currentMean);
		}
		//We put the activePoints in the values array
		switch (distribution){
		case 0:
			//We put them in random places in sets of "stability" members
			//We only check that the active point is free, but not the stability next ones
			//So it is likely that we are going to delete a previous value, this yields 
			//slightly different stability values
			for (int i=0; i<activePoints; i++){
				boolean placed = false;
				while (!placed){
					int point = (int) (Math.random()*clusters);
					if (values[point]== -1) {
						for (int j = 0; j<stability && j+point<clusters; j++) values[point+j]= active[i];
						placed = true;
					}
				}
			}
			break;
		case 1:
			//If the probability is above the mean there is more tendency to be on the first half
			for (int i=0; i<activePoints; i++){
				boolean placed = false;
				while (!placed){
					int point=0;
					float chance = (float) Math.random();
					//There is a 60% of probability that the point is in the first half if value>mean
					if (active[i]>=mean && chance>0.4) {
						point = (int) (Math.random()*clusters)/2;
					}
					else
						//There is a 60% of probability that the point is in the second half if value<mean
						if (active[i]<mean && chance>0.4) {
							point = clusters-(int) (Math.random()*clusters)/2;
						}
						else 
							point = (int) (Math.random()*clusters);
					if (values[point]== -1) {
						for (int j = 0; j<stability && j+point<clusters; j++) values[point+j]= active[i];
						placed = true;
					}
				}
			}
			break;
		case 2:
			//If the probability is above the mean there is more tendency to be on the second half
			for (int i=0; i<activePoints; i++){
				boolean placed = false;
				while (!placed){
					int point=0;
					float chance = (float) Math.random();
					//There is a 60% of probability that the point is in the first half if value<mean
					if (active[i]<=mean && chance>0.4) {
						point = (int) (Math.random()*clusters)/2;
					}
					else
						//There is a 60% of probability that the point is in the second half if value>mean
						if (active[i]>mean && chance>0.4) {
							point = clusters-(int) (Math.random()*clusters)/2;
						}
						else 
							point = (int) (Math.random()*clusters);
					if (values[point]== -1) {
						for (int j = 0; j<stability && j+point<clusters; j++) values[point+j]= active[i];
						placed = true;
					}
				}
			}
			break;
		case 3:
			//If the probability is above the mean there is more tendency to be near the center
			for (int i=0; i<activePoints; i++){
				boolean placed = false;
				while (!placed){
					int point=0;
					float chance = (float) Math.random();
					//There is a 60% of probability that the point is in the center if value>mean
					if (active[i]>=mean && chance>0.4) {
						point = (int) (Math.random()*clusters)/2+clusters/4;
					}
					else
						//There is a 60% of probability that the point is near the beginning or the end if value<mean
						if (active[i]<mean && chance>0.4) {
							int chance2 = (int) Math.random()*2; //to locate it at the beginning or at the end randomly
							if (chance2<1) 
								point = (int) (Math.random()*clusters)/4;
							else 
								point = (int) (Math.random()*clusters)/4+3*clusters/4;
						}
						else 
							point = (int) (Math.random()*clusters);
					if (values[point]== -1) {
						for (int j = 0; j<stability && j+point<clusters; j++) values[point+j]= active[i];
						placed = true;
					}
				}
			}
			break;
		case 4:
			//If the probability is above the mean there is more tendency to be near the extremes
			for (int i=0; i<activePoints; i++){
				boolean placed = false;
				while (!placed){
					int point=0;
					float chance = (float) Math.random();
					//There is a 60% of probability that the point is near center if value<mean 
					if (active[i]<=mean && chance>0.4) {
						point = (int) (Math.random()*clusters)/2+clusters/4;
					}
					else
						//There is a 60% of probability that the point is in the extremes if value>mean
						if (active[i]>mean && chance>0.4) {
							int chance2 = (int) Math.random()*2; //to locate it at the beginning or at the end randomly
							if (chance2<1) 
								point = (int) (Math.random()*clusters)/4;
							else 
								point = (int) (Math.random()*clusters)/4+3*clusters/4;
						}
						else 
							point = (int) (Math.random()*clusters);
					if (values[point]== -1) {
						for (int j = 0; j<stability && j+point<clusters; j++) values[point+j]= active[i];
						placed = true;
					}
				}
			}
			break;
		}
		//Uncomment to see where the active values have been placed
		//for (int i=0; i<clusters; i++) System.out.print(values[i]+",");
		//Now we fill the rest of the values interpolating, we assume first value is 0.0
		int pos1=0; 
		if (values[0]== -1) values[0]=0;
		if (values[clusters-1]== -1) values[clusters-1]=0;
		float val1=0, step;
		for (int i=1; i<clusters; i++){
			//We look for the first value != -1
			if (values[i]!=-1){
				step = (values[i]-val1)/(i-pos1);
				for (int j=pos1; j<i; j++) {
					values[j+1] = values[j]+step;
					if (values[j+1]<0.0001) values[j+1]=0;
					//A bug in Europa prevents probabilities equal to 1
					if (values[j+1]>=0.999) values[j+1]=0.999F;
				}
				pos1 = i;
				val1 = values[i];
			}
		}
		System.out.println();
	//	for (int i=0; i<clusters; i++) System.out.print(values[i]+",");
		return values;
	}

	public void writeFile (float [] cluster){
		try {
			PrintWriter pw = new PrintWriter(file);
			pw.println ("LUT _plume_err = new LUT(plume, -1, 0.0);"); 
			pw.println ("LUT _plume0 = new LUT(plume, 0, 0.0);");
			for (int i=0; i< clusters; i++)
				pw.println ("LUT _inl"+(i+1)+" = new LUT(plume, "+(i+1)+", "+cluster[i]+");");
			pw.close();
			System.out.println ("Cluster created successfuly in file "+file);
		}
		catch (IOException e){
			System.out.println(e);
		}
	}
	private float calculateMean (float values []){
		float mean=0;
		for (int i=0; i<activePoints; i++) mean = mean + values[i];
		mean = mean / activePoints;
		return mean;
	}
	public static void main(String[] args) throws IOException {
		// TODO Auto-generated method stub
		CreateCluster test = new CreateCluster();
		test.asking();
		test.writeFile(test.createClusters());
		
	}

}	