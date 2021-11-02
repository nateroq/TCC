--#region Setting parameters

--param:set('WPNAV_SPEED',500) -- Horizontal speed
--param:set('WPNAV_ACCEL',125) -- Horizontal acceleration
--param:set('SIM_WIND_DIR',0) -- Horizontal Wind direction
--param:set('SIM_WIND_DIR_Z',0) -- Vertical Wind direction
--param:set('SIM_WIND_SPD',0) -- Horizontal Wind speed
--param:set('SCR_HEAP_SIZE',1000000) -- Memomy to run scripts

--#endregion

function MvV(m1,m2)
  local R = {};
  for i = 1, 3, 1 do
    R[i] = (m1[i][1]*m2[1])+(m1[i][2]*m2[2])+(m1[i][3]*m2[3]);
  end

  return R;
end

function print_val(val,strg, typ)
  if typ == 0 then
    gcs:send_text(0, string.format("%s: [%.2f, %.2f, %.2f]",strg, val[1], val[2], val[3]));
  elseif typ == 1 then
    gcs:send_text(0, string.format("%s:", strg));
    for i = 1, 3, 1 do
      gcs:send_text(0, string.format("[%.2f, %.2f, %.2f]", val[i][1], val[i][2], val[i][3]));
    end
    gcs:send_text(0, string.format("\n"));
  else
    gcs:send_text(0, string.format("%s: %.2f ", strg, val));
  end
end

function update () -- periodic function that will be called
    if mission:state() == mission.MISSION_RUNNING then -- check to see if mission is running
      if (mission:get_current_nav_index() > 1  and 
            mission:get_current_nav_index() < (mission:num_commands() -1))  then -- if it is not takeoff or RTL
        
        --#region P WI WI+1 DEFINITION

        if mission:get_current_nav_index() == 2 then
          wi = (ahrs:get_home()):get_vector_from_origin_NEU(); --the first waypoint is home position
          wi:x(-wi:x());wi:y(-wi:y()); -- direction change beacause of conversion
        else
          wi = wi1; -- otherwise wi is the previous wi+1
        end
        p = ahrs:get_relative_position_NED_home(); -- fetch the current position of the vehicle
        local nextwp = mission:get_item(mission:get_current_nav_index()); -- fetch the current wi+1
        local wi1loc = Location(); -- create Localtion for easy NED convertion
        wi1loc:lat(nextwp:x());wi1loc:lng(nextwp:y());wi1loc:alt(nextwp:z()); -- fill with coords
        wi1 = (ahrs:get_home()):get_distance_NED(wi1loc); -- convert wi+1 to NED
        wi1:z(-wi1loc:alt()); -- reconfiguring alt beacause conversion

        --#endregion

        --#region FIRST STEP: ALPHA

        local alpha = math.atan( (wi1:y()) - (wi:y()),(wi1:x()) - (wi:x()) );

        --#endregion

        --#region SECOND STEP: Rz
        local Rz = {{(math.cos(alpha)),(-math.sin(alpha)),0}, 
                    {(math.sin(alpha)),(math.cos(alpha)),0}, 
                    {0,0,1}};

        --#endregion

        --#region THIRD STEP: Rz * WI

        local WiR = MvV(Rz,{wi:x(),wi:y(),wi:z()});
        
        --#endregion

        --#region FOURTH STEP: Rz * WI+1
        
        local Wi1R = MvV(Rz,{wi1:x(),wi1:y(),wi1:z()});

        --#endregion

        --#region FIFTH STEP: BETA
        local bet_z = (WiR[3] - Wi1R[3]);
        local bet_x = (Wi1R[1] - WiR[1]);
        bet_x = bet_x*bet_x;
        local bet_y = (Wi1R[2] - WiR[2]);
        bet_y = bet_y*bet_y;
        local bet_sqr = math.sqrt((bet_x+bet_y));
        local beta = math.atan(bet_z,bet_sqr);

        --#endregion

        --#region SIXTH STEP: Ry'
        local Ry = {{(math.cos(beta)),0,(math.sin(beta))}, 
                    {0,1,0}, 
                    {(-math.sin(beta)),1,(math.cos(beta))}};

        --#endregion

        --#region SEVENTH STEP: P-WI
        local pminusw = {(p:x() - wi:x()),(p:y() - wi:y()),(p:z() - wi:z())};

        --#endregion

        --#region EIGTH STEP: Ry' * Rz * P - WI
        local erro = MvV(Ry,(MvV(Rz,pminusw)));
        --#endregion

        --#region SAVE AS CSV

        --#endregion

        --#region PRINTS
        --print_val(alpha,'Alpha'); -- for number don't send a third argument
        --print_val({wi:x(),wi:y(),wi:z()},'WI',0); --for Vector3() types send as table with 0 as third argument
        print_val(erro,'Erro:',0); -- for table also use 0 as third argument
        --print_val(Rz,'Rz',1); -- for table of tables use 1 as third argument
        --#endregion
        
      end
      
    end
    
    return update, 500 -- request "update" to be rerun again 1000 milliseconds (1 second) from now
  end
  return update, 500   -- request "update" to be the first time 1000 milliseconds (1 second) after script is loaded