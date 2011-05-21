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
