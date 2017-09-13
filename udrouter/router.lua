#!/usr/bin/lua

local script_dir = (debug.getinfo(1).source:match("@?(.*/)")) or ""
package.path = package.path..';'..script_dir.."?.lua"
local rs232 = require("luars232")
local socket = require("socket")
local bindechex = require("bindechex")
local posix = require("posix")
local version = require("version") 
local logic = require("logic") 

--/*---------------------------------------------------------------------------*/--

device_group = {}
DEVICE_GROUP_BUTTON_SWITCH     =     "00"
device_group[DEVICE_GROUP_BUTTON_SWITCH]		   =     "Button/switch"

DEVICE_GROUP_SENSORS           =     "01"
device_group[DEVICE_GROUP_SENSORS]				   =     "Sensor"

DEVICE_GROUP_MOTION_SENSOR     =     "02"
device_group[DEVICE_GROUP_MOTION_SENSOR]		   =     "Motion sensor"

DEVICE_GROUP_OPEN_SENSORS      =     "03"
device_group[DEVICE_GROUP_OPEN_SENSORS]			   =     "Door open sensor"

DEVICE_GROUP_METERS            =     "04"
device_group[DEVICE_GROUP_METERS]				   =     "Meter"

DEVICE_GROUP_RELAY             =     "05"
device_group[DEVICE_GROUP_RELAY]				   =     "Relay"

DEVICE_GROUP_DIMMER            =     "06"
device_group[DEVICE_GROUP_DIMMER]				   =     "Dimmer"

DEVICE_GROUP_LIGHT             =     "07"
device_group[DEVICE_GROUP_LIGHT]				   =     "Light"

DEVICE_GROUP_RGB_LIGHT         =     "08"
device_group[DEVICE_GROUP_RGB_LIGHT]			   =     "RGB light"

DEVICE_GROUP_BRIDGE_CONVERTER  =     "09"
device_group[DEVICE_GROUP_BRIDGE_CONVERTER]		   =     "Bridge/Converter"

DEVICE_GROUP_OTHER             =     "FF"
device_group[DEVICE_GROUP_OTHER]				   =     "Other device"

--/*---------------------------------------------------------------------------*/--

device_ability = {}

DEVICE_ABILITY_NONE            =     "00"

DEVICE_ABILITY_BUTTON          =     "01"
device_ability[DEVICE_ABILITY_BUTTON] = "Button/switch"

DEVICE_ABILITY_TEMPERATURE     =     "02"
device_ability[DEVICE_ABILITY_TEMPERATURE] = "Temperature sensor"

DEVICE_ABILITY_HUMIDITY        =     "03"
device_ability[DEVICE_ABILITY_HUMIDITY] = "Humidity sensor"

DEVICE_ABILITY_PRESSURE        =     "04"
device_ability[DEVICE_ABILITY_PRESSURE] = "Pressure sensor"

DEVICE_ABILITY_LIGHT_SENSOR    =     "05"
device_ability[DEVICE_ABILITY_LIGHT_SENSOR] = "Light sensor"

DEVICE_ABILITY_NOISE_SENSOR    =     "06"
device_ability[DEVICE_ABILITY_NOISE_SENSOR] = "Noise sensor"

DEVICE_ABILITY_MOTION_SENSOR   =     "07"
device_ability[DEVICE_ABILITY_MOTION_SENSOR] = "Motion sensor"

DEVICE_ABILITY_RESERVED1       =     "08"

DEVICE_ABILITY_C02_SENSOR      =     "09"
device_ability[DEVICE_ABILITY_C02_SENSOR] = "CO2 sensor"

DEVICE_ABILITY_CO_SENSOR       =     "0A"
device_ability[DEVICE_ABILITY_CO_SENSOR] = "CO sensor"

DEVICE_ABILITY_GAS_SENSOR      =     "0B"
device_ability[DEVICE_ABILITY_GAS_SENSOR] = "GAS sensor"

DEVICE_ABILITY_POWER_METER     =     "0C"
device_ability[DEVICE_ABILITY_POWER_METER] = "Power/voltage meter"

DEVICE_ABILITY_RADIATION_METER       =     "0D"
device_ability[DEVICE_ABILITY_RADIATION_METER] = "Radiation meter"

DEVICE_ABILITY_RESERVED3       =     "0E"
DEVICE_ABILITY_RESERVED4       =     "0F"
DEVICE_ABILITY_RESERVED5       =     "10"

DEVICE_ABILITY_RELAY           =     "11"
device_ability[DEVICE_ABILITY_RELAY] = "Relay"


DEVICE_ABILITY_DIMMER          =     "12"
device_ability[DEVICE_ABILITY_DIMMER] = "Dimmer"

DEVICE_ABILITY_RESERVED6       =     "13"
DEVICE_ABILITY_RESERVED7       =     "14"
DEVICE_ABILITY_RESERVED8       =     "15"
DEVICE_ABILITY_RESERVED9       =     "16"
DEVICE_ABILITY_RESERVED10      =     "17"

DEVICE_ABILITY_LED             =     "18"
device_ability[DEVICE_ABILITY_LED] = "LED indicator"

--/*---------------------------------------------------------------------------*/--

device_button_events = {}

DEVICE_ABILITY_BUTTON_EVENT_CLICK        =   "01"
device_button_events[DEVICE_ABILITY_BUTTON_EVENT_CLICK] = "click"

DEVICE_ABILITY_BUTTON_EVENT_LONG_CLICK   =   "02"
device_button_events[DEVICE_ABILITY_BUTTON_EVENT_LONG_CLICK] = "longclick"

DEVICE_ABILITY_BUTTON_EVENT_ON           =   "03"
device_button_events[DEVICE_ABILITY_BUTTON_EVENT_ON] = "on"

DEVICE_ABILITY_BUTTON_EVENT_OFF          =   "04"
device_button_events[DEVICE_ABILITY_BUTTON_EVENT_OFF] = "off"

--/*---------------------------------------------------------------------------*/--

device_motionsensor_events = {}

DEVICE_ABILITY_MOTION_SENSOR_EVENT_MOTION        =   "01"
device_motionsensor_events[DEVICE_ABILITY_MOTION_SENSOR_EVENT_MOTION] = "motion"

DEVICE_ABILITY_MOTION_SENSOR_EVENT_NO_MOTION   =   "02"
device_motionsensor_events[DEVICE_ABILITY_MOTION_SENSOR_EVENT_NO_MOTION] = "nomotion"

--/*---------------------------------------------------------------------------*/--

device_relay_commands = {}

DEVICE_ABILITY_RELAY_COMMAND_ON        =   "01"
device_relay_commands[DEVICE_ABILITY_RELAY_COMMAND_ON] = "on"

DEVICE_ABILITY_RELAY_COMMAND_OFF   =   "00"
device_relay_commands[DEVICE_ABILITY_RELAY_COMMAND_OFF] = "off"

DEVICE_ABILITY_RELAY_COMMAND_TOGGLE   =   "80"
device_relay_commands[DEVICE_ABILITY_RELAY_COMMAND_TOGGLE] = "toggle"

--/*---------------------------------------------------------------------------*/--

device_sleep_type = {}

DEVICE_SLEEP_TYPE_NORMAL             =           "01"
device_sleep_type[DEVICE_SLEEP_TYPE_NORMAL] = "Non-sleep"

DEVICE_SLEEP_TYPE_LEAF               =           "02"
device_sleep_type[DEVICE_SLEEP_TYPE_LEAF] = "Leaf mode"

--/*---------------------------------------------------------------------------*/--

device_message_type = {}

DEVICE_MESSAGE_HIGH_TEMPERATYRE             					=           "01"
device_message_type[DEVICE_MESSAGE_HIGH_TEMPERATYRE] 			= "Warning: high temperature"

DEVICE_MESSAGE_LOW_VOLTAGE               			            =           "02"
device_message_type[DEVICE_MESSAGE_LOW_VOLTAGE] 	            = "Warning: low voltage"

DEVICE_MESSAGE_HIGH_CURRENT               			         	=           "03"
device_message_type[DEVICE_MESSAGE_HIGH_CURRENT] 		     	= "Warning: high current"

DEVICE_MESSAGE_LOW_POWER               			                =           "04"
device_message_type[DEVICE_MESSAGE_LOW_POWER] 	             	= "Warning: low power"

DEVICE_MESSAGE_ERROR_ON_RELAY               		      	    =           "05"
device_message_type[DEVICE_MESSAGE_ERROR_ON_RELAY] 	         	= "Warning: error on relay"

DEVICE_MESSAGE_ERROR_OFF_RELAY               		      	    =           "06"
device_message_type[DEVICE_MESSAGE_ERROR_OFF_RELAY] 	     	= "Warning: error off relay"
 
DEVICE_MESSAGE_OTA_SPI_NOTACTIVE             					=           "07"
device_message_type[DEVICE_MESSAGE_OTA_SPI_NOTACTIVE] 			= "OTA: External flash not active"

DEVICE_MESSAGE_OTA_NOT_DELIVERED_CHUNK               			=           "08"
device_message_type[DEVICE_MESSAGE_OTA_NOT_DELIVERED_CHUNK] 	= "OTA: Chunk not delivered"

DEVICE_MESSAGE_OTA_NONCORRECT_CRC               				=           "09"
device_message_type[DEVICE_MESSAGE_OTA_NONCORRECT_CRC] 			= "OTA: Non-correct image CRC"

DEVICE_MESSAGE_OTA_BAD_GOLDEN_IMAGE               			    =           "0A"
device_message_type[DEVICE_MESSAGE_OTA_BAD_GOLDEN_IMAGE] 		= "OTA: Bad golden image"

DEVICE_MESSAGE_OTA_SPI_ERASE_IN_PROGRESS               		    =           "0B"
device_message_type[DEVICE_MESSAGE_OTA_SPI_ERASE_IN_PROGRESS] 	= "OTA: SPI flash erase in progress"

DEVICE_MESSAGE_OTA_UPDATE_SUCCESS               		        =           "0C"
device_message_type[DEVICE_MESSAGE_OTA_UPDATE_SUCCESS] 	        = "OTA: Update success"

--/*---------------------------------------------------------------------------*/--

DATA_TYPE_FIRMWARE_COMMAND_NEW_FW              =         "01" --Сообщение о наличии новой прошивки
DATA_TYPE_FIRMWARE_COMMAND_chunk_REQ           =         "02" --Запрос пакета с частью прошивки

PROTOCOL_VERSION_V1            =     "01"
DEVICE_VERSION_V1              =     "01"

UART_PROTOCOL_VERSION_V1       =     "01"
UART_PROTOCOL_VERSION_V2       =     "02"
UART_PROTOCOL_VERSION_V3       =     "03"

UART_PV2_START_MQ = "011616161610"
UART_NONE_DATA = "FFFFFFFFFFFFFFFFFFFF"
UART_FF_DATA = "FF"

UART_NONE_100_DATA = "FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF"
UART_NONE_25_DATA = "FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF"
UART_NONE_20_DATA = "FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF"
UART_NONE_10_DATA = "FFFFFFFFFFFFFFFFFFFF"

UART_NONE_26_DATA = "FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF"


VOLTAGE_PRESCALER = 16

--/*---------------------------------------------------------------------------*/--

DATA_TYPE_JOIN                            =              "01" --Запрос на включение в сеть
DATA_TYPE_SENSOR_DATA                     =              "02" --Данные с датчиков устройства
DATA_TYPE_CONFIRM                 	      =              "03" --Подтверждение запроса на включение в сеть
DATA_TYPE_PONG                            =              "04" --Подтверждение доставки пакета
DATA_TYPE_COMMAND                         =              "05" --Команды возможностям устройства
DATA_TYPE_STATUS                          =              "06" --Пакет со статусными данными
DATA_TYPE_GET_STATUS                      =              "07" --Запрос статуса(не реализовано)
DATA_TYPE_SETTINGS                        =              "08" --Команда настройки параметров
DATA_TYPE_MESSAGE                         =              "09" --Сообщения
DATA_TYPE_SET_TIME                        =              "0A" --Команда установки времени(не реализовано)
DATA_TYPE_SET_SCHEDULE                    =              "0B" --Команда установки расписания(не реализовано)
DATA_TYPE_FIRMWARE                        =              "0C" --Данные для OTA
DATA_TYPE_UART                            =              "0D" --Команда с данными UART
DATA_TYPE_FIRMWARE_CMD                    =              "0E" --Команды OTA

--/*---------------------------------------------------------------------------*/--

switch_mini_door = "fd00:0000:0000:0000:0212:4b00:0c47:4b82"
switch_mini_bed = "fd00:0000:0000:0000:0212:4b00:0c47:4a82"
switch_mini_table = "fd00:0000:0000:0000:0212:4b00:0c47:3b04"
switch_wc = "fd00:0000:0000:0000:0212:4b00:0c47:4b05"
switch_main_room = "fd00:0000:0000:0000:0212:4b00:0c47:3a00"
switch_kitchen = "fd00:0000:0000:0000:0212:4b00:0c47:4880"

relay_main_room_table = "fd00:0000:0000:0000:0212:4b00:0c47:4a85"
relay_main_room = "fd00:0000:0000:0000:0212:4b00:0c47:3e00"
relay_hall = "fd00:0000:0000:0000:0212:4b00:0c47:3c84"
relay_kitchen = "fd00:0000:0000:0000:0212:4b00:0c47:4a02"
relay_bathroom = "fd00:0000:0000:0000:0212:4b00:0c47:4802"
relay_wc = "fd00:0000:0000:0000:0212:4b00:0c47:4886"

motion_sensor_hall = "fd00:0000:0000:0000:0212:4b00:0c47:4602"

--/*---------------------------------------------------------------------------*/--

devices_names = {}
devices_names[switch_mini_door] = "switch_mini_door"
devices_names[switch_mini_bed] = "switch_mini_bed"
devices_names[switch_mini_table] = "switch_mini_table"
devices_names[switch_wc] = "switch_wc"
devices_names[switch_main_room] = "switch_main_room"
devices_names[switch_kitchen] = "switch_kitchen"

devices_names[relay_main_room_table] = "relay_main_room_table"
devices_names[relay_main_room] = "relay_main_room"
devices_names[relay_hall] = "relay_hall"
devices_names[relay_kitchen] = "relay_kitchen"
devices_names[relay_bathroom] = "relay_bathroom"
devices_names[relay_wc] = "relay_wc"

devices_names[motion_sensor_hall] = "motion_sensor_hall"

--/*---------------------------------------------------------------------------*/--

api_keys = {} -- keys for thingspeak
api_keys[switch_mini_door] = "9U3175EF4NWFLCHU"
api_keys[switch_mini_bed] = "PCHJXTUODC0LS2PW"
api_keys[switch_mini_table] = "F80QAU2U0RM47IFG"
api_keys[switch_wc] = "GHZLBBRFOFFYSUP3"
api_keys[switch_main_room] = "6P06S2I81X412JIR"
api_keys[switch_kitchen] = "LSGSFREU9GHK30YR"

api_keys[relay_main_room_table] = "RB2SEV381GNAI14R"
api_keys[relay_main_room] = "99DOTCKKCPGHYB3U"
api_keys[relay_hall] = "FKGCJJC68X5SOO7I"
api_keys[relay_kitchen] = "6AZZVI8SFXQBFDHW"
api_keys[relay_bathroom] = "YTAAIPT9281MH1ZJ"
api_keys[relay_wc] = "9NTCZ3CB7CTQVMH2"

--/*---------------------------------------------------------------------------*/--

local ver = version.git
local uart_version = UART_PROTOCOL_VERSION_V1
local pid_file = "/tmp/run/unwired_router.pid"
local port_name = "/dev/ttyATH0"
local pid = posix.getpid()
local start_time = 0
local state = "all_on"
local main_cycle_permit = 1
local device_list = {}

--/*---------------------------------------------------------------------------*/--

function string.fromhex(str)
    local str = string.gsub(str, " ", "") 
    return (str:gsub('..', function (cc)
        return string.char(tonumber(cc, 16))
    end))
end

function string.tohex(str)
    return (str:gsub('.', function (c)
        return string.format('%02X ', string.byte(c))
    end))
end

--/*---------------------------------------------------------------------------*/--

raw_print = print

function print(data)
	io.write(data or "")
	io.write("\n")
	io.flush()
end

function print_n(data)
	io.write(data or "")
	io.flush()
end

--/*---------------------------------------------------------------------------*/--

function colors(color)
	if (color == "red") then
		io.write(string.char(27,91,51,49,109))
	elseif (color == "none") then
		io.write(string.char(27,91,48,109))
	end
end

--/*---------------------------------------------------------------------------*/--

function update_ts_channels(address, voltage, uptime)
	if api_keys[address] == nil then return end
	local command = 'wget --no-check-certificate --wait=20 --random-wait --dns-timeout=5 --connect-timeout=10 --tries=0 --output-document=- "https://api.thingspeak.com/update?api_key='..api_keys[address]..'&field2='..voltage..'&field1='..uptime..'" &>/dev/null &'
	os.execute(command)
end

--/*---------------------------------------------------------------------------*/--

function led(state)
	if (state == "on") then
		os.execute("echo 1 > /sys/devices/platform/leds-gpio/leds/unwone\:green\:eth1_rxtx/brightness")
	elseif (state == "off") then
		os.execute("echo 0 > /sys/devices/platform/leds-gpio/leds/unwone\:green\:eth1_rxtx/brightness")
	end
end

--/*---------------------------------------------------------------------------*/--

function ipv6_adress_parse(ipv6_adress)
	local adress_capturing = "(%w%w)(%w%w):(%w%w)(%w%w):(%w%w)(%w%w):(%w%w)(%w%w):(%w%w)(%w%w):(%w%w)(%w%w):(%w%w)(%w%w):(%w%w)(%w%w)"
	local _
	local a = {}

	_, end_1, a[1],a[2],a[3],a[4],a[5],a[6],a[7],a[8],a[9],a[10],a[11],a[12],a[13],a[14],a[15],a[16]  = string.find(ipv6_adress, adress_capturing)
	if (end_1 ~= nil) then
		return a[1]..a[2]..a[3]..a[4]..a[5]..a[6]..a[7]..a[8]..a[9]..a[10]..a[11]..a[12]..a[13]..a[14]..a[15]..a[16]
	else
		print("IV6P: Adress parse error "..ipv6_adress)
		return nil
	end
end

--/*---------------------------------------------------------------------------*/--

function data_cut(all_data, segment_len)
  local max_len = segment_len
  local data_len = string.len(all_data)
  local i = 1
  local data = {}
  repeat 
    data_len = string.len(all_data)
    local data_sub = string.sub(all_data, 0, max_len)
    all_data = string.sub(all_data, max_len+1)
    data[i] = data_sub
    i = i + 1
  until (data_len < max_len+1)
  return data
end

--/*---------------------------------------------------------------------------*/--

function uart_send(bin_data)
	local packet_data = bin_data

	local uart_packet_size = #bin_data
	local max_packet_size = 1500

	if (#bin_data > max_packet_size) then
		print("Too big packet size")
		return
	end

	local uart_packet_size_hex = bindechex.Dec2Hex(uart_packet_size)
	if (#uart_packet_size_hex == 1) then uart_packet_size_hex = "000"..uart_packet_size_hex
	elseif (#uart_packet_size_hex == 2) then uart_packet_size_hex = "00"..uart_packet_size_hex
	elseif (#uart_packet_size_hex == 3) then uart_packet_size_hex = "0"..uart_packet_size_hex
	end

	uart_packet_size_hex_b1 = string.sub(uart_packet_size_hex, 1, 2)
	uart_packet_size_hex_b2 = string.sub(uart_packet_size_hex, 3, 4)

	local preamble = UART_PV2_START_MQ:fromhex()..UART_PROTOCOL_VERSION_V3:fromhex()..uart_packet_size_hex_b2:fromhex()..uart_packet_size_hex_b1:fromhex()..UART_FF_DATA:fromhex()
	bin_data = preamble..bin_data

	--print("Send packet:\n"..preamble:tohex().."\n"..packet_data:tohex().."\nPacket size: "..#packet_data..", uart packet size: "..#bin_data.."\n")

	local table_segments = data_cut(bin_data, 25)
	for i = 1, #table_segments do 
		p:write(table_segments[i])
		socket.sleep(0.006)
	end 
end

--/*---------------------------------------------------------------------------*/--

function send_command_to_ability(ipv6_adress, ability_target, ability_number, ability_state)
	local adress = ipv6_adress_parse(ipv6_adress)
	local bin_data = ""

	bin_data = bin_data..adress:fromhex()
	bin_data = bin_data..PROTOCOL_VERSION_V1:fromhex()
	bin_data = bin_data..DEVICE_VERSION_V1:fromhex()
	bin_data = bin_data..DATA_TYPE_COMMAND:fromhex()
	bin_data = bin_data..ability_target:fromhex()
	bin_data = bin_data..ability_number:fromhex()
	bin_data = bin_data..ability_state:fromhex()
	uart_send(bin_data)
	--print("Processing time "..(math.ceil(socket.gettime()*1000 - start_time)).." ms")
	--start_time = socket.gettime()*1000
	socket.sleep(0.08)
end

--/*---------------------------------------------------------------------------*/--

function send_uart_data_to_ability(ipv6_adress, returned_data_lenth, data_lenth, payload)
	local adress = ipv6_adress_parse(ipv6_adress)
	local bin_data = ""
	bin_data = bin_data..adress:fromhex()
	bin_data = bin_data..PROTOCOL_VERSION_V1:fromhex()
	bin_data = bin_data..DEVICE_VERSION_V1:fromhex()
	bin_data = bin_data..DATA_TYPE_UART:fromhex()
	bin_data = bin_data..data_lenth:fromhex()
	bin_data = bin_data..returned_data_lenth:fromhex()
	bin_data = bin_data..(payload[1] or "FF"):fromhex()
	bin_data = bin_data..(payload[2] or "FF"):fromhex()
	bin_data = bin_data..(payload[3] or "FF"):fromhex()
	bin_data = bin_data..(payload[4] or "FF"):fromhex()
	bin_data = bin_data..(payload[5] or "FF"):fromhex()
	bin_data = bin_data..(payload[6] or "FF"):fromhex()
	bin_data = bin_data..(payload[7] or "FF"):fromhex()
	bin_data = bin_data..(payload[8] or "FF"):fromhex()
	bin_data = bin_data..(payload[9] or "FF"):fromhex()
	bin_data = bin_data..(payload[10] or "FF"):fromhex()
	bin_data = bin_data..(payload[11] or "FF"):fromhex()
	bin_data = bin_data..(payload[12] or "FF"):fromhex()
	bin_data = bin_data..(payload[13] or "FF"):fromhex()
	bin_data = bin_data..(payload[14] or "FF"):fromhex()
	bin_data = bin_data..(payload[15] or "FF"):fromhex()
	bin_data = bin_data..(payload[16] or "FF"):fromhex()
	uart_send(bin_data)
end

--/*---------------------------------------------------------------------------*/--

function send_firmware_new_fw_cmd_to_node(ipv6_adress, table_segments)
	local adress = ipv6_adress_parse(ipv6_adress)
	local chunk_quantity = #table_segments

	local chunk_quantity_hex = bindechex.Dec2Hex(chunk_quantity)
	if (#chunk_quantity_hex == 1) then chunk_quantity_hex = "000"..chunk_quantity_hex
	elseif (#chunk_quantity_hex == 2) then chunk_quantity_hex = "00"..chunk_quantity_hex
	elseif (#chunk_quantity_hex == 3) then chunk_quantity_hex = "0"..chunk_quantity_hex
	end

	--raw_print(chunk_quantity_hex, chunk_quantity)

	local bin_data = ""
	bin_data = bin_data..adress:fromhex()
	bin_data = bin_data..PROTOCOL_VERSION_V1:fromhex()
	bin_data = bin_data..DEVICE_VERSION_V1:fromhex()
	bin_data = bin_data..DATA_TYPE_FIRMWARE_CMD:fromhex()
	bin_data = bin_data..DATA_TYPE_FIRMWARE_COMMAND_NEW_FW:fromhex()

	bin_data = bin_data..chunk_quantity_hex:fromhex() --2 bytes

	uart_send(bin_data)
end

--/*---------------------------------------------------------------------------*/--

function send_firmware_chunk_to_node(ipv6_adress, firmware_bin_chunk)
	local adress = ipv6_adress_parse(ipv6_adress)

	local bin_data = ""
	bin_data = bin_data..adress:fromhex()
	bin_data = bin_data..PROTOCOL_VERSION_V1:fromhex()
	bin_data = bin_data..DEVICE_VERSION_V1:fromhex()
	bin_data = bin_data..DATA_TYPE_FIRMWARE:fromhex()

	bin_data = bin_data..UART_FF_DATA:fromhex()
	bin_data = bin_data..UART_FF_DATA:fromhex()

	bin_data = bin_data..UART_FF_DATA:fromhex() --reserved
	bin_data = bin_data..UART_FF_DATA:fromhex() --reserved
	
	bin_data = bin_data..firmware_bin_chunk

	uart_send(bin_data)
end

--/*---------------------------------------------------------------------------*/--

function send_relay_command(ipv6_adress, relay_number, state)
	--print("Relay command processing start: +"..(socket.gettime()*1000 - start_time).." ms")

	local device_name = devices_names[ipv6_adress] or ipv6_adress
	print("Send command to device: "..device_name..", relay:"..relay_number..", state:"..state)
	local ability_state, ability

	if (state == device_relay_commands[DEVICE_ABILITY_RELAY_COMMAND_ON]) then
		ability_state = DEVICE_ABILITY_RELAY_COMMAND_ON
	elseif (state == device_relay_commands[DEVICE_ABILITY_RELAY_COMMAND_OFF]) then
		ability_state = DEVICE_ABILITY_RELAY_COMMAND_OFF
	elseif (state == device_relay_commands[DEVICE_ABILITY_RELAY_COMMAND_TOGGLE]) then
		ability_state = DEVICE_ABILITY_RELAY_COMMAND_TOGGLE
	end

	if (relay_number == 1) then
		ability_number = tostring("01")
	elseif (relay_number == 2) then
		ability_number = tostring("02")
	end
	
	if (ability_state ~= nil) then
		send_command_to_ability(ipv6_adress, DEVICE_ABILITY_RELAY, ability_number, ability_state)
	else
		print("Non-valid ability state!")
	end
end

--/*---------------------------------------------------------------------------*/--

function motion_sensor_data_processing(ipv6_adress, sensor_number, sensor_event)
	local current_event = device_motionsensor_events[sensor_event]
	print(" MDPM: Motion sensor: "..sensor_number..", event: "..current_event)
	
	logic.motion_sensor_event_handler(ipv6_adress, sensor_number, current_event)
end


--/*---------------------------------------------------------------------------*/--

function button_data_processing(ipv6_adress, sensor_number, sensor_event)
	local button_name = string.upper(tostring(sensor_number):fromhex())
	local current_event = device_button_events[sensor_event]
	print(" BDPM: Button: "..button_name..", event: "..device_button_events[sensor_event])

	logic.button_sensor_event_handler(ipv6_adress, button_name, current_event)
end

--/*---------------------------------------------------------------------------*/--

function sensor_data_processing(ipv6_adress, data)
	--print("Sensor data processing module")
	--print("Sensor data processing start: +"..(socket.gettime()*1000 - start_time).." ms")
	local number_ability = data.b1 or "no number_ability"
	local sensor_number = data.b3 or "no sensor_number"
	local sensor_event = data.b4 or "no sensor_event"
	local sensor_name = device_ability[number_ability] or "Not found ability description: "..number_ability

	local device_name = devices_names[ipv6_adress] or ipv6_adress
	print("\nSDPM: Device: "..device_name..", sensor type: "..sensor_name)

	if (number_ability == DEVICE_ABILITY_MOTION_SENSOR) then
		motion_sensor_data_processing(ipv6_adress, sensor_number, sensor_event)
	end

	if (number_ability == DEVICE_ABILITY_BUTTON) then
		button_data_processing(ipv6_adress, sensor_number, sensor_event)
	end
end

--/*---------------------------------------------------------------------------*/--

function join_data_processing(ipv6_adress, data)
	--print("Join data processing module")
	if (flag_non_status_join_message ~= nil) then
		return
	end

	local current_device_group = data.b1 or "no device_group"
	local current_sleep_type = data.b2 or "no sleep_type"
	local ability_1 = data.b3 or "no ability_1"
	local ability_2 = data.b4 or "no ability_2"
	local ability_3 = data.b5 or "no ability_3"
	local ability_4 = data.b6 or "no ability_4"

	local device_group_name = device_group[current_device_group] or current_device_group
	local device_sleep_name = device_sleep_type[current_sleep_type] or current_sleep_type
	local device_name = devices_names[ipv6_adress] or ipv6_adress
	print("\nJDPM: Join packet from "..device_name..", device group: "..device_group_name..", sleep type: "..device_sleep_name)
	--print("\n")
end

--/*---------------------------------------------------------------------------*/--

function getu16(data, pos) -- reads a 16 bit value of the string data at position pos
	local high, low = string.byte(data, pos, pos+1)
	local val = high*256 + low -- no bit fiddling in lua since everything is a double internally
	return val
end

--/*---------------------------------------------------------------------------*/--

function status_data_processing(ipv6_adress, data)
	--print("Status data processing module")
	if (flag_non_status_join_message ~= nil) then
		return
	end

	local ipv6_adress_parent_short = data.b1..data.b2..":"..data.b3..data.b4..":"..data.b5..data.b6..":"..data.b7..data.b8

	local uptime = tonumber(bindechex.Hex2Dec((data.b12 or 00)..(data.b11 or 00)..(data.b10 or 00)..(data.b9 or 00)))
	local voltage = (bindechex.Hex2Dec(data.b16 or 00)*VOLTAGE_PRESCALER/1000)
	local temp = bindechex.Hex2Dec(data.b15 or 00)
	local parent_rssi_raw = tonumber(bindechex.Hex2Dec((data.b14 or 00)..(data.b13 or 00)))
	local version = bindechex.Hex2Dec(data.b17 or 00).."."..bindechex.Hex2Dec(data.b18 or 00)
	local ota_flag = data.b19
	if ota_flag == "01" then ota_flag = "Active" else ota_flag = "Non-active" end

	if parent_rssi_raw > 32768 then
		parent_rssi_raw = parent_rssi_raw - 65536
	end

	parent_rssi = string.format("%d, %i, %u", parent_rssi_raw, parent_rssi_raw, parent_rssi_raw) or "no rssi"
	parent_rssi = parent_rssi_raw
	local device_name = devices_names[ipv6_adress] or ipv6_adress
	print("\nSDPM: Status packet from "..device_name..":")
	print(" Version: "..version)
	print(" OTA: "..ota_flag)
	print(" Parent adress: "..ipv6_adress_parent_short)
	print(" Uptime: "..uptime.."s")
	print(" Parent RSSI: "..parent_rssi.."dbm")
	print(" Temp: "..temp.."C")
	print(" Voltage: "..voltage.." v")
	--print("\n")

	update_ts_channels(ipv6_adress, voltage, uptime/60/60/24)
end

--/*---------------------------------------------------------------------------*/--

function message_data_processing(ipv6_adress, data)
	--print("Message status processing module")

	if (message_data_processing_flag_n == nil) then
		print_n("\n")
		message_data_processing_flag_n = " "		
	end
	
	if (data.b1 == DEVICE_MESSAGE_OTA_SPI_ERASE_IN_PROGRESS) then
		print_n("\r"..device_message_type[data.b1].." from "..ipv6_adress..": page "..(bindechex.Hex2Dec(data.b2)).."/24")
	elseif (data.b1 == DEVICE_MESSAGE_OTA_UPDATE_SUCCESS) then
		print("MDPM: message packet from "..ipv6_adress..": "..device_message_type[data.b1])
		print_n("\n")
		main_cycle_permit = 0
	else
		print("MDPM: message packet from "..ipv6_adress..":")
		print(" "..device_message_type[data.b1])
		print_n("\n")
	end
	
end

--/*---------------------------------------------------------------------------*/--

function fw_cmd_data_processing(ipv6_adress, data)
	if (fw_cmd_data_processing_flag_n == nil) then
		print_n("\n")
		fw_cmd_data_processing_flag_n = " "
	end

	local chunk_number_c_style = tonumber(bindechex.Hex2Dec((data.b3 or 00)..(data.b2 or 00)))
	local chunk_number_lua_style = chunk_number_c_style + 1

	if (ota_image_table_segments == nil) then
		return
	end

	if (#ota_image_table_segments < chunk_number_c_style) then
		print("Bad chunk_number")
		os.exit(0)
	end

	firmware_bin_chunk = ota_image_table_segments[chunk_number_lua_style]

	if (firmware_bin_chunk == null) then
		print("Chunk null")
		os.exit(0)
	end

	local chunk_quantity = #ota_image_table_segments
	print_n("\rFW OTA processing module: send to node chunk "..chunk_number_lua_style.."/"..chunk_quantity)
	send_firmware_chunk_to_node(ipv6_adress, firmware_bin_chunk)

	if (tonumber(chunk_number_lua_style) == tonumber(chunk_quantity)) then
		print(". End chunks\n")
		--os.exit(0)
	end
end

--/*---------------------------------------------------------------------------*/--

function packet_processing_see_adresses(a)
	local ipv6_adress = a[1]..a[2]..":"..a[3]..a[4]..":"..a[5]..a[6]..":"..a[7]..a[8]..":"..a[9]..a[10]..":"..a[11]..a[12]..":"..a[13]..a[14]..":"..a[15]..a[16]

	for i = 1, #device_list do 
		if (ipv6_adress == device_list[i]) then
			return
		end
	end
	device_list[#device_list+1] = ipv6_adress
	print(ipv6_adress)
end


--/*---------------------------------------------------------------------------*/--

function packet_processing(a, data)
	--print("Packet processing module")
	--print("Packet processing start: +"..(socket.gettime()*1000 - start_time).." ms")
	local ipv6_adress = a[1]..a[2]..":"..a[3]..a[4]..":"..a[5]..a[6]..":"..a[7]..a[8]..":"..a[9]..a[10]..":"..a[11]..a[12]..":"..a[13]..a[14]..":"..a[15]..a[16]
	if (data.p_version == PROTOCOL_VERSION_V1 and data.dev_version == DEVICE_VERSION_V1) then
		if data.d_type == DATA_TYPE_JOIN then
		 	join_data_processing(ipv6_adress, data)

		elseif data.d_type == DATA_TYPE_SENSOR_DATA then
			sensor_data_processing(ipv6_adress, data)

		elseif data.d_type == DATA_TYPE_STATUS then
			status_data_processing(ipv6_adress, data)

		elseif data.d_type == DATA_TYPE_MESSAGE then
			message_data_processing(ipv6_adress, data)

		elseif data.d_type == DATA_TYPE_FIRMWARE_CMD then
			fw_cmd_data_processing(ipv6_adress, data)

		elseif data.d_type == DATA_TYPE_CONFIRM then
			print("PPM: Data type: DATA_TYPE_CONFIRM from "..ipv6_adress)

		elseif data.d_type == DATA_TYPE_PING then
			print("PPM: Data type: DATA_TYPE_PING from "..ipv6_adress)

		elseif data.d_type == DATA_TYPE_COMMAND then
			print("PPM: Data type: DATA_TYPE_COMMAND from "..ipv6_adress)

		else
			print(string.format("Unsupported protocol type(%s)", tostring(data.d_type)))

		end
	else
		print(string.format("Unsupported protocol version(%s) or device version(%s)", tostring(data.p_version), tostring(data.dev_version)))
		print("Adress: "..ipv6_adress)
	end
end

--/*---------------------------------------------------------------------------*/--

function packet_parse(packet, packet_processing_alt)
	--print("Packet parse module")
	--print("Packet parse start: +"..(socket.gettime()*1000 - start_time).." ms")
	local adress_capturing_all = "(%w%w%w%w%w%w%w%w%w%w%w%w%w%w%w%w%w%w%w%w%w%w%w%w%w%w%w%w%w%w%w%w)"
	local data_capturing_all = "(%w%w%w%w%w%w%w%w%w%w%w%w%w%w%w%w%w%w%w%w%w%w%w%w%w%w%w%w%w%w%w%w%w%w%w%w%w%w%w%w%w%w%w%w%w%w)"
	local _, _, adress, raw_data = string.find(packet, "DAGROOTRAW1"..adress_capturing_all..data_capturing_all.."RAWEND")
	if (adress ~= nil and raw_data ~= nil) then
		local _ = {}
		local a = {}
		local d = {}
		local adress_capturing = "(%w%w)(%w%w)(%w%w)(%w%w)(%w%w)(%w%w)(%w%w)(%w%w)(%w%w)(%w%w)(%w%w)(%w%w)(%w%w)(%w%w)(%w%w)(%w%w)"
		local data_capturing = "(%w%w)(%w%w)(%w%w)(%w%w)(%w%w)(%w%w)(%w%w)(%w%w)(%w%w)(%w%w)(%w%w)(%w%w)(%w%w)(%w%w)(%w%w)(%w%w)(%w%w)(%w%w)(%w%w)(%w%w)(%w%w)(%w%w)(%w%w)"
		
		_, end_1, a[1],a[2],a[3],a[4],a[5],a[6],a[7],a[8],a[9],a[10],a[11],a[12],a[13],a[14],a[15],a[16]  = string.find(adress, adress_capturing)
		_, end_2, d.p_version,d.dev_version,d.d_type,d.b1,d.b2,d.b3,d.b4,d.b5,d.b6,d.b7,d.b8,d.b9,d.b10,d.b11,d.b12,d.b13,d.b14,d.b15,d.b16,d.b17,d.b18,d.b19,d.b20 = string.find(raw_data, data_capturing)
		if (end_1 ~= nil and end_2 ~= nil) then
			if (packet_processing_alt == nil) then
				packet_processing(a, d)
			else
				packet_processing_alt(a, d)
			end
		else
			print(string.format("Not parse adress(%s) or data(%s) in packet"), tostring(end_1), tostring(end_2))
			print(packet)
			print(adress)
			print(adress_capturing)
			print(raw_data)
			print(data_capturing)
		end
	else
		print("Not parse packet: "..packet)
	end
end

--/*---------------------------------------------------------------------------*/--

function send_uart_command(command, address, delay)
	local cmd_on = {"1B", "31", "32", "33", "2B"}

	local cmd_off = {"1B", "30", "30", "30", "2B"}

	if (command == "on") then
		send_uart_data_to_ability(address, "0B", "05", cmd_on)
	elseif (command == "off") then
		send_uart_data_to_ability(address, "0B", "05", cmd_off)
	elseif (command == "on_off") then
		while true do
			send_uart_data_to_ability(address, "0B", "05", cmd_on)
			socket.sleep(delay or 10)
			send_uart_data_to_ability(address, "0B", "05", cmd_off)
			socket.sleep(delay or 10)
		end
	end
end

--/*---------------------------------------------------------------------------*/--

function add_byte_to_buffer(buffer, byte)
	table.insert(buffer, byte)
	while (#buffer > 95) do
		table.remove(buffer, 1)
	end
end

function clean_buffer(buffer)
	for i = 1, #buffer do 
		table.insert(buffer, 0)
	end
end

function get_buffer(buffer)
	return table.concat(buffer)
end

--/*---------------------------------------------------------------------------*/--

function port_monitor()
 	while 1 do
		_, data_read = p:read(1, 200)
		if (data_read ~= nil) then	
			io.write(data_read)
			io.flush()		
		end
	end
end

--/*---------------------------------------------------------------------------*/--

function data_cut(all_data, segment_len)
  local max_len = segment_len
  local data_len = string.len(all_data)
  local i = 1
  local data = {}
  repeat 
    data_len = string.len(all_data)
    local data_sub = string.sub(all_data, 0, max_len)
    all_data = string.sub(all_data, max_len+1)
    data[i] = data_sub
    i = i + 1
  until (data_len < max_len+1)
  return data
end
	
--/*---------------------------------------------------------------------------*/--

function main_cycle(limit, adresses_print_mode)
	local _, data_read, packet, message
	local end_time, now_time = 0, 0
	local buffer = {}
	if (limit ~= nil) then
		now_time = socket.gettime()*1000
		end_time = now_time+(limit*1000)
	end

 	while (main_cycle_permit == 1) do
		_, data_read = p:read(1, 200)
		if (data_read ~= nil) then			
			--print_n(data_read)			
			add_byte_to_buffer(buffer, data_read)
			local buffer_state = get_buffer(buffer)
			_, _, packet = string.find(buffer_state, "(DAGROOTRAW1"..('.'):rep(78).."RAWEND)")
			if (packet ~= nil) then
				led("on")
				if adresses_print_mode ~= nil then
					packet_parse(packet, packet_processing_see_adresses)
				else
					packet_parse(packet)
				end
				led("off")
			end
			_, _, message = string.find(buffer_state, "(UDM:.+\n)")
			if (message ~= nil) then
				led("on")
				print(message)
				clean_buffer(buffer)
				led("off")
			end
		end
		if (limit ~= nil) then
			now_time = socket.gettime()*1000
			if (now_time > end_time) then
				main_cycle_permit = 0
			end
		end
	end
end

--/*---------------------------------------------------------------------------*/--

function firmware_update(image_file, address)	
	local handle, err = io.open(image_file,"r")
	if (err ~= nil) then
		print("Error open file")
		return
	end
	local image_file_bin_data = handle:read("*a")
	handle:close()
	local chunk_size = 600
	ota_image_table_segments = data_cut(image_file_bin_data, chunk_size)
	chunk_quantity = #ota_image_table_segments

	local addition_ff = (("FF"):fromhex()):rep(chunk_size - #ota_image_table_segments[chunk_quantity])
	ota_image_table_segments[chunk_quantity] = ota_image_table_segments[chunk_quantity]..addition_ff

	send_firmware_new_fw_cmd_to_node(address, ota_image_table_segments)
end

--/*---------------------------------------------------------------------------*/--

local f,err = io.open(pid_file,"w")
if not f then
	print(err, f)
else
	f:write(pid)
	f:close()
end

colors("red")
print("RPL-router version "..ver..", uart protocol version: "..uart_version.."\n")
colors("none")

e, p = rs232.open(port_name)
if e ~= rs232.RS232_ERR_NOERROR then
	print(string.format("can't open serial port '%s', error: '%s'\n",
			port_name, rs232.error_tostring(e)))
	return
end

assert(p:set_baud_rate(rs232.RS232_BAUD_115200) == rs232.RS232_ERR_NOERROR)
assert(p:set_data_bits(rs232.RS232_DATA_8) == rs232.RS232_ERR_NOERROR)
assert(p:set_parity(rs232.RS232_PARITY_NONE) == rs232.RS232_ERR_NOERROR)
assert(p:set_stop_bits(rs232.RS232_STOP_1) == rs232.RS232_ERR_NOERROR)
assert(p:set_flow_control(rs232.RS232_FLOW_OFF)  == rs232.RS232_ERR_NOERROR)

--/*---------------------------------------------------------------------------*/--

if (arg[1] == "uart_asuno_test") then
	if (arg[2] == nil or arg[3] == nil) then
		print("use:\trouter.lua uart_asuno_test fd00:0000:0000:0000:0212:4b00:0c47:4a85 on/off/on_off 5(on_off cycle delay in sec, default 10)")
		return
	end
	local command, address, delay = arg[3], arg[2], arg[4]
	send_uart_command(command, address, delay)
elseif (arg[1] == "fw") then
	if (arg[2] == nil or arg[3] == nil) then
		print("use:\trouter.lua fw image_file fd00:0000:0000:0000:0212:4b00:0f0a:8b9b fd00:0000:0000:0000:0212:4b00:0f0a:8b9b ...")
		return
	end
	flag_non_status_join_message = "true"
	local image_file = arg[2]
	for i = 3, #arg do 
		main_cycle_permit = 1
		fw_cmd_data_processing_flag_n = nil
		firmware_update(image_file, arg[i])
		main_cycle()
		colors("red")
		print("Update device "..arg[i].."("..(i-2).."/"..(#arg-2)..") success!\n")
		colors("none")
	end 
	return
elseif (arg[1] == "device_list") then
	main_cycle(nil, 1)
elseif (arg[1] == "main") then
	main_cycle()
elseif (arg[1] == "monitor") then
	port_monitor()
else
	print("Use:\trouter.lua main \t\tstart main loop(data parse/show)\n\trouter.lua fw \tsend firmware file to node\n\trouter.lua uart_asuno_test \tsend uart asuno command to node\n\trouter.lua monitor \t\tstart port monitor\n")
end

