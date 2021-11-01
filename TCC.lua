--Setting parameters

--param:set('WPNAV_SPEED',500) -- Horizontal speed
--param:set('WPNAV_ACCEL',125) -- Horizontal acceleration
--param:set('SIM_WIND_DIR',0) -- Horizontal Wind direction
--param:set('SIM_WIND_DIR_Z',0) -- Vertical Wind direction
--param:set('SIM_WIND_SPD',0) -- Horizontal Wind speed
--param:set('SCR_HEAP_SIZE',1000000) -- Memomy to run scripts

function update () -- periodic function that will be called
    if mission:state() == mission.MISSION_RUNNING then -- check to see if mission is running
      p = ahrs:get_relative_position_NED_home(); -- fetch the current position of the vehicle
      gcs:send_text(0, string.format("p  --  x:%.1f y:%.1f z:%.1f", p:x(), p:y(), p:z()));
      if (mission:get_current_nav_index() > 1  and 
            mission:get_current_nav_index() < (mission:num_commands() -1))  then -- if it is not take off or RTL
        if mission:get_current_nav_index() == 2 then
          wi = (ahrs:get_home()):get_vector_from_origin_NEU(); --the first waypoint is home position
        else
          wi = wi1; -- otherwise wi is the previous wi+1
        end
        local nextwp = mission:get_item(mission:get_current_nav_index()); -- fetch the current wi+1
        local wi1loc = Location(); -- create Localtion for easy NED convertion
        wi1loc:lat(nextwp:x());wi1loc:lng(nextwp:y());wi1loc:alt(nextwp:z()); -- fill with coords
        wi1 = (ahrs:get_home()):get_distance_NED(wi1loc); -- convert wi+1 to NED
        wi1:z(-wi1loc:alt()); -- reconfiguring alt beacause conversion
        gcs:send_text(0, string.format("w1   - x:%.1f y:%.1f z:%.1f", wi:x() , wi:y(), wi:z()));
        gcs:send_text(0, string.format("wi+1 - x:%.1f y:%.1f z:%.1f", wi1:x(), wi1:y(), wi1:z()));
        --local alpha = 

         --[=====[ 
        local pminusw = {(current_pos.x-wi.x),(current_pos.y-wi.y),(current_pos.z-wi.z)}; -- p-w
        gcs:send_text(0, string.format("p-w - Lat:%.1f Long:%.1f Alt:%.1f", pminusw[1], pminusw[2], pminusw[3]));
        --local verror = 
        --]=====]
        
      end
      
    end
    
    return update, 1000 -- request "update" to be rerun again 1000 milliseconds (1 second) from now
  end
  return update, 1000   -- request "update" to be the first time 1000 milliseconds (1 second) after script is loaded