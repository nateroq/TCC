--Setting parameters

--param:set('WPNAV_SPEED',500) -- Horizontal speed
--param:set('WPNAV_ACCEL',125) -- Horizontal acceleration
--param:set('SIM_WIND_DIR',0) -- Horizontal Wind direction
--param:set('SIM_WIND_DIR_Z',0) -- Vertical Wind direction
--param:set('SIM_WIND_SPD',0) -- Horizontal Wind speed

function update () -- periodic function that will be called
    if mission:state() == mission.MISSION_RUNNING then -- check to see if mission is running
      current_pos = ahrs:get_relative_position_NED_origin(); -- fetch the current position of the vehicle
      current_pos:z(-1*current_pos:z()); -- fromm ned to neu
      gcs:send_text(0, string.format("current position - x:%.1f y:%.1f z:%.1f", current_pos:x(), current_pos:y(), current_pos:z()));
      if mission:get_current_nav_index() < 8 then -- if it is the first waypoint
        local homeloc = ahrs:get_home();  -- fetch the home position of the vehicle
        wi = homeloc:get_vector_from_origin_NEU(); --the first waypoint is home position
        local nextwp = mission:get_item(mission:get_current_nav_index()); -- fetch the current wi+1
        local wi1loc = Location(); -- create localtion for easy neu convertion
        wi1loc:lat(nextwp:x());
        wi1loc:lng(nextwp:y());
        wi1loc:alt(nextwp:z());
        wi1 = wi1loc:get_vector_from_origin_NEU(); -- convert wi1 to neu
        --gcs:send_text(0, string.format("FIRST WAYPOINT"));
        --gcs:send_text(0, string.format("wi - Lat:%.1f Long:%.1f Alt:%.1f", wi:x(), wi:y(), wi:z()));
        gcs:send_text(0, string.format("wi1 - Lat:%.1f Long:%.1f Alt:%.1f", wi1:x() , wi1:y(), wi1:z()));
        --local alpha = 

         --[=====[ 
        local pminusw = {(current_pos.x-wi.x),(current_pos.y-wi.y),(current_pos.z-wi.z)}; -- p-w
        gcs:send_text(0, string.format("p-w - Lat:%.1f Long:%.1f Alt:%.1f", pminusw[1], pminusw[2], pminusw[3]));
        --local verror = 
        
      else
        wi = wi1;      -- current waypoint is reached waypoint
        --test = mission:get_current_do_cmd_id();
        --gcs:send_text(0, string.format("WP: %d",test))
        --wi1 = mission:get_item(test); -- fetch the target location -- next waypoint is next target location
        --gcs:send_text(0, string.format("NEXT WAYPOINTS"))
        --gcs:send_text(0, string.format("wi - Lat:%.1f Long:%.1f Alt:%.1f", wi:x(), wi:y(), wi:z()))
        --gcs:send_text(0, string.format("wi1 - Lat:%.1f Long:%.1f Alt:%.1f", wi1:x(), wi1:y(), wi1:z()))
        --local pminusw = current_pos:get_distance_NED(wi); -- p-w
        --gcs:send_text(0, string.format("p-w - Lat:%.1f Long:%.1f Alt:%.1f", pminusw:x(), pminusw:y(), pminusw:z()))
        --]=====]
      end
      
    end
    

    return update, 1000 -- request "update" to be rerun again 1000 milliseconds (1 second) from now
  end
  return update, 1000   -- request "update" to be the first time 1000 milliseconds (1 second) after script is loaded