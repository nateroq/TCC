-- Example of loading a mission from the SD card using Scripting and executing it.

-- File that contains the mission starting with takeoff and ending with a return to launch
-- Put your file inside ArduCopter folder
local navigation_file = 'way_1.txt'
-- Set as true if you want to repeat the mission continuously
local repeat_mission = true
local mission_loaded = false
local mission_started = false
local mission_ended = false

local function read_mission(file_name)
  -- Open file
  file = assert(io.open(file_name), 'Could open :' .. file_name)

  -- check header
  assert(string.find(file:read('l'),'QGC WPL 110') == 1, file_name .. ': incorrect format')

  -- clear any existing mission
  assert(mission:clear(), 'Could not clear current mission')

  -- read each line and write to mission
  local item = mavlink_mission_item_int_t()
  local index = 0
  while true do

    local data = {}
    for i = 1, 12 do
      data[i] = file:read('n')
      if data[i] == nil then
        if i == 1 then
          gcs:send_text(6, 'loaded mission: ' .. file_name)
          return -- got to the end of the file
        else
          mission:clear() -- clear part loaded mission
          error('failed to read file')
        end
      end
    end

    item:seq(data[1])
    item:frame(data[3])
    item:command(data[4])
    item:param1(data[5])
    item:param2(data[6])
    item:param3(data[7])
    item:param4(data[8])
    item:x(data[9]*10^7)
    item:y(data[10]*10^7)
    item:z(data[11])

    if not mission:set_item(index,item) then
      mission:clear() -- clear part loaded mission
      error(string.format('failed to set mission item %i',index))
    end
    index = index + 1
  end

end
--[=====[ 
function update()
  local current_pos = ahrs:get_position()
  local origin = ahrs:get_origin()
  local mission_state = mission:state()

  if not mission_loaded then
    read_mission(navigation_file)
    mission_loaded = true
  end

  -- wait to have solutions for position
  -- gets stuck and never continues - giving up
  -- if not ahrs:home_is_set() then
  --   gcs:send_text(6, 'Waiting for AHRS solutions')
  --   return update, 5000
  -- end
  -- gcs:send_text(6, 'AHRS initialised')


  -- Debugging messages, uncomment to see the values if desirable
  -- gcs:send_text(6, 'Inside update')
  -- gcs:send_text(6, 'Current mode ' .. vehicle:get_mode())
  -- if current_pos then
  --   gcs:send_text(6, ' current_pos Lat ' .. current_pos:lat() ..
  --                 ' Long ' .. current_pos:lng() ..
  --                 ' Alt ' .. current_pos:alt())
  -- end
  -- if mission_state then
  --   gcs:send_text(6, ' mission_state ' .. mission_state)
  -- end

  -- start mission
  if (not mission_started and not arming:is_armed()) then
    gcs:send_text(6, 'Arming')
    arming:arm()
  elseif (arming:is_armed() and vehicle:get_mode() == 0) then
    gcs:send_text(6, 'Mode auto')
    if not vehicle:set_mode(3) then
      return update, 10000
    else
      mission_started = true
    end
  end

  if mission_state == mission.MISSION_COMPLETE then
    gcs:send_text(6, 'Mission Ended!')
    if repeat_mission then
      gcs:send_text(6, 'Starting again.. ')
      mission:set_current_cmd(0)
      mission_started = false
      vehicle:set_mode(0)
    end
  end


  return update, 1000
end

return update()
--]=====]