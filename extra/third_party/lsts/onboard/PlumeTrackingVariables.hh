/*
 * PlumeTrackerReactor.hh
 *
 *  Created on: May 24, 2017
 *      Author: pcooksey
 */

namespace TREX {
  /** @brief plume indicator plug-in
   *
   * This namespace is shared by all LSTS reactors/plugins
   * @ingroup lsts
   *
   * @author Philip Cooksey <pcooksey@andrew.cmu.edu>
   */
  namespace LSTS {
    
    
        //Latitude, in degrees, of river mouth , longitude=
        double river_lat = 0.7180188974;//41.13945

        //Longitude, in degrees, of river mouth
        double river_lon = -0.1514448372;//-8.67715

        //Maximum depth, in meters, for yoyo profiles
        double min_depth = 0;

        //Minimum depth, in meters, for yoyo profiles
        double max_depth = 5;
        
        //Depth, in meters, for surface plume tracking
        double surface_depth = 1;

        //Speed to travel at during yoyo profiles
        double yoyo_speed = 1300;
        
        //Speed units to use (RPM, m/s)
        std::string speed_units = "RPM";

        //Number of yoyos to perform on each side of the plume
        int yoyo_count = 5;

        //Start angle, in degrees
        double start_ang = -180;

        //End angle, in degrees
        double end_ang = -45;

        //Variation, in degrees, between survey angles
        double angle_inc = 10;

        //Minimum distance from river mouth
        double min_dist = 100;

        //Maximum distance from river mouth
        double max_dist = 500;
        
        //Distance outside the plume
        double outside_plume_dist = 100;

        //Salinity Threshold
        double salinity = 30.0;
        
        //Number of salinity values to average
        int salinity_count = 2;
        
        //Use Simulated Plume
        bool simulated_plume = true;
        
        //Distance of simulated plume
        double plume_dist = 1000;       

        //Seconds to idle at each vertex
        int wait_secs = 60;

        //DUNE Host Address
        std::string host_addr = "127.0.0.1";

        //DUNE Host Port (TCP)
        int host_port = 6003;
        
        //Minutes before termination
        int mins_timeout = 60;
        
        //DUNE plan to execute right after termination
        std::string end_plan = "rendezvous";
        
        //Maximum time underwater
        int mins_underwater = 15;
  }
  
}