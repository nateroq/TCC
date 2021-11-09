--#region Flags and Globals
-- Parameters that can be changed
experiment_id = 4; -- flag for experiment
max_number_replications = 10; -- total number of replications for experiment
navigation_file = 'Missions_TCC/openangles.waypoints'; --mission file
param:set('WPNAV_SPEED',1000); -- Horizontal speed
param:set('WPNAV_ACCEL',250); -- Horizontal acceleration
param:set('SIM_WIND_SPD',0); -- Horizontal Wind speed
flag_ctrl_mission = 1;
-- Global Variables that should not be changed
replication_number = 0; -- replication number of the experiment
repeat_mission = true; -- bool flag for mission repetition
mission_loaded = false; -- bool flag for mission load
mission_started = false; -- bool flag to star mission
mission_ended = true; -- bool flag to end mission
wp = mission:get_current_nav_index(); -- flag for wi=wi+1 control
vetor_erro = {0,0,0}; -- mear error vector
verror_size = 0; -- size of mean error vector
written_file_replication = false;
first_position = 0;
--#endregion

function experiment_control() -- function that controls the creation of csv for the whole experiment
  mean_file_name = "logs/exp/exp" .. tostring(experiment_id) .. "_mean.csv"; --file name changes for each experiment
  mean_file = io.open(mean_file_name, "a"); --append/create .csv
  mean_file:write('Replication, Ex(m), Ey(m), Ez(m)\n'); --csv headers
  mean_file:flush(); --save to file

  all_errors_file_name = "logs/exp/exp" .. tostring(experiment_id) .. "_all_errors.csv"; --file name changes for each experiment
  all_errors_file = io.open(all_errors_file_name, "a"); --append/create .csv
  all_errors_file:write('Ex(m), Ey(m), Ez(m)\n'); --csv headers
  all_errors_file:flush(); --save to file
end

--#region Setting parameters for missions
function mission_param()
  if experiment_id == 2 then
    param:set('SIM_WIND_SPD',5);
    param:set('WPNAV_SPEED',1000);
    param:set('WPNAV_ACCEL',250);
  end
  if experiment_id == 3 then
    param:set('SIM_WIND_SPD',10);
    param:set('WPNAV_SPEED',1000);
    param:set('WPNAV_ACCEL',250);
  end
  if experiment_id == 4 then
    navigation_file = 'Missions_TCC/closedangles.waypoints';
    param:set('SIM_WIND_SPD',5);
    param:set('WPNAV_SPEED',1000);
    param:set('WPNAV_ACCEL',250);
  end
  if experiment_id == 5 then
    navigation_file = 'Missions_TCC/smallsquare.waypoints';
    param:set('SIM_WIND_SPD',5);
    param:set('WPNAV_SPEED',1000);
    param:set('WPNAV_ACCEL',250);
  end
  if experiment_id == 6 then
    param:set('SIM_WIND_SPD',5);
    param:set('WPNAV_SPEED',1000);
    param:set('WPNAV_ACCEL',125);
  end
  if experiment_id == 7 then
    param:set('SIM_WIND_SPD',5);
    param:set('WPNAV_SPEED',1000);
    param:set('WPNAV_ACCEL',300);
  end
  if experiment_id == 8 then
    param:set('SIM_WIND_SPD',5);
    param:set('WPNAV_SPEED',500);
    param:set('WPNAV_ACCEL',250);
  end
  if experiment_id == 9 then
    param:set('SIM_WIND_SPD',5);
    param:set('WPNAV_SPEED',1500);
    param:set('WPNAV_ACCEL',250);
  end
  local param1 = param:get('SIM_WIND_SPD');
  local param2 = param:get('WPNAV_SPEED');
  local param3 = param:get('WPNAV_ACCEL');
  gcs:send_text(6, string.format('Wind:  %i',param1));
  gcs:send_text(6, string.format('Speed: %i',param2));
  gcs:send_text(6, string.format('Accel: %i',param3));
end
--#endregion

--#region WRITE DATA TO OPEN FILE
function write_to_file(file, interesting_data)
  if not file then
    error("Could not open file");
  end
  -- separate with comas and add a carriage return
  -- make sure file is upto date
  file:write(table.concat(interesting_data,", ") .. "\n");
  file:flush();

end
--#endregion

--#region READ MISSION
function read_mission(mission_file_name)
  -- Open file
  mission_file = assert(io.open(mission_file_name), 'Could open :' .. mission_file_name);

  -- check header
  assert(string.find(mission_file:read('l'),'QGC WPL 110') == 1, mission_file_name .. ': incorrect format');

  -- clear any existing mission
  assert(mission:clear(), 'Could not clear current mission');

  -- read each line and write to mission
  local item = mavlink_mission_item_int_t();
  local index = 0;
  while true do

    local data = {};
    for i = 1, 12 do
      data[i] = mission_file:read('n');
      if data[i] == nil then
        if i == 1 then
          gcs:send_text(6, 'loaded mission: ' .. mission_file_name);
          return -- got to the end of the file
        else
          mission:clear() -- clear part loaded mission
          error('failed to read file');
        end
      end
    end

    item:seq(data[1]);
    item:frame(data[3]);
    item:command(data[4]);
    item:param1(data[5]);
    item:param2(data[6]);
    item:param3(data[7]);
    item:param4(data[8]);
    item:x(data[9]*10^7);
    item:y(data[10]*10^7);
    item:z(data[11]);

    if not mission:set_item(index,item) then
      mission:clear(); -- clear part loaded mission
      error(string.format('failed to set mission item %i',index));
    end
    index = index + 1;
  end

end
--#endregion

--#region CALCULATER THE MATRIX * VECTOR
function MvV(m1,m2)
  local R = {};
  for i = 1, 3, 1 do
    R[i] = (m1[i][1]*m2[1])+(m1[i][2]*m2[2])+(m1[i][3]*m2[3]);
  end
  return R
end
--#endregion

--#region PRINT VALUES
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
--#endregion

function update () -- periodic function that will be called
  --#region START MISSION
  if not mission_loaded then
    read_mission(navigation_file);
    mission_loaded = true;
  end
  -- start mission
  if (not mission_started and not arming:is_armed()) then
    gcs:send_text(6, 'Arming');
    arming:arm();
  elseif (arming:is_armed() and vehicle:get_mode() == 0 and repeat_mission ) then
    gcs:send_text(6, 'Mode auto');
    if not vehicle:set_mode(3) then
      return update, 10000
    end
  end

  --#endregion

  if mission:state() == mission.MISSION_RUNNING then -- check to see if mission is running
    if (mission_ended == true) then
      mission_ended = false;
      mission_started = true;;
      vetor_erro = {0,0,0};
      verror_size = 0;
      first_position = ahrs:get_relative_position_NED_home();
      replication_number = replication_number+1; --flag for file change increments
      gcs:send_text(6, 'Replication number ' .. replication_number);
      written_file_replication = false;
      write_to_file(all_errors_file, {replication_number});
    end


    if (mission:get_current_nav_index() > 1  and
        mission:get_current_nav_index() < (mission:num_commands() -1))  then -- if it is not takeoff or RTL

    --#region P WI WI+1 DEFINITION
    if mission:get_current_nav_index() == 2 then
      wi = first_position; --the first waypoint where airship is currently situated
      --(ahrs:get_home()):get_vector_from_origin_NEU(); 
      wi:z(-10); -- we only measure on air

    elseif wp ~= mission:get_current_nav_index()  then
      wi = wi1; -- otherwise wi is the previous wi+1
      wp = mission:get_current_nav_index(); -- only when waypoint passes
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
    local Rz = {{(math.cos(alpha)),(math.sin(alpha)),0},
                {(-math.sin(alpha)),(math.cos(alpha)),0},
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
    local Ry = {{(math.cos(beta)),0,(-math.sin(beta))},
                {0,1,0},
                {(math.sin(beta)),0,(math.cos(beta))}};
    --#endregion

    --#region SEVENTH STEP: P-WI
    local pminusw = {(p:x() - wi:x()),(p:y() - wi:y()),(p:z() - wi:z())};
    --#endregion

    --#region EIGTH STEP: Ry' * Rz * P - WI
    local erro = MvV(Ry,(MvV(Rz,pminusw)));
    vetor_erro = {vetor_erro[1]+math.abs(erro[1]),vetor_erro[2]+math.abs(erro[2]),vetor_erro[3]+math.abs(erro[3])};
    verror_size = verror_size+1;
    --#endregion

    --#region SAVE AS CSV
    write_to_file(all_errors_file, erro);
    --#endregion


    --#region PRINTS
    --print_val(alpha,'Alpha'); -- for number don't send a third argument
    --print_val({wi:x(),wi:y(),wi:z()},'WI',0); --for Vector3() types send as table with 0 as third argument
    --print_val(erro,'Erro:',0); -- for table also use 0 as third argument
    --print_val(Rz,'Rz',1); -- for table of tables use 1 as third argument
    --#endregion

    end
  end

  if (mission:state() == mission.MISSION_COMPLETE and not mission_started and not vehicle:set_mode(0)) then -- if mission ended save mission mean to csv
    return update, 1000
  elseif mission:state() == mission.MISSION_COMPLETE then -- if mission ended save mission mean to csv
    if not written_file_replication then
      vetor_erro = {vetor_erro[1]/verror_size,vetor_erro[2]/verror_size,vetor_erro[3]/verror_size};
      local csv_data = {replication_number, vetor_erro[1], vetor_erro[2], vetor_erro[3]};
      write_to_file(mean_file, csv_data);
      written_file_replication = true;
    end
    if (replication_number == max_number_replications) then
      repeat_mission = false
    end

    if repeat_mission then
      mission:set_current_cmd(0);
      mission_started = false;
      vehicle:set_mode(0);
    end
    mission_ended = true;

  end
  return update, 500 -- request "update" to be rerun again 1000 milliseconds (1/2 second) from now
end

mission_param(); --call to create as soon as the script starts
experiment_control(); --call to create as soon as the script starts
return update, 500   -- request "update" to be the first time 1000 milliseconds (1/2 second) after script is loaded
