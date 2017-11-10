#!/usr/bin/lua
--[[
/*
 * Copyright (c) 2016, Unwired Devices LLC - http://www.unwireddevices.com/
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Unwired Devices nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */ ]]--

local script_dir = (debug.getinfo(1).source:match("@?(.*/)")) or ""
package.path = package.path..';'..script_dir.."?.lua"
local rs232 = require("rs232")
local socket = require("socket")
local bindechex = require("bindechex")
local posix = require("posix")
local lanes = require "lanes".configure()
local linda = lanes.linda()
local bit = require "bit"

--/*---------------------------------------------------------------------------*/--

local device_group = {}
local DEVICE_GROUP_BUTTON_SWITCH     =     "00"
device_group[DEVICE_GROUP_BUTTON_SWITCH]		   =     "Button/switch"

local DEVICE_GROUP_SENSORS           =     "01"
device_group[DEVICE_GROUP_SENSORS]				   =     "Sensor"

local DEVICE_GROUP_MOTION_SENSOR     =     "02"
device_group[DEVICE_GROUP_MOTION_SENSOR]		   =     "Motion sensor"

local DEVICE_GROUP_OPEN_SENSORS      =     "03"
device_group[DEVICE_GROUP_OPEN_SENSORS]			   =     "Door open sensor"

local DEVICE_GROUP_METERS            =     "04"
device_group[DEVICE_GROUP_METERS]				   =     "Meter"

local DEVICE_GROUP_RELAY             =     "05"
device_group[DEVICE_GROUP_RELAY]				   =     "Relay"

local DEVICE_GROUP_DIMMER            =     "06"
device_group[DEVICE_GROUP_DIMMER]				   =     "Dimmer"

local DEVICE_GROUP_LIGHT             =     "07"
device_group[DEVICE_GROUP_LIGHT]				   =     "Light"

local DEVICE_GROUP_RGB_LIGHT         =     "08"
device_group[DEVICE_GROUP_RGB_LIGHT]			   =     "RGB light"

local DEVICE_GROUP_BRIDGE_CONVERTER  =     "09"
device_group[DEVICE_GROUP_BRIDGE_CONVERTER]		   =     "Bridge/Converter"

local DEVICE_GROUP_OTHER             =     "FF"
device_group[DEVICE_GROUP_OTHER]				   =     "Other device"

--/*---------------------------------------------------------------------------*/--

local device_ability = {}

local DEVICE_ABILITY_NONE            =     "00"

local DEVICE_ABILITY_BUTTON          =     "01"
device_ability[DEVICE_ABILITY_BUTTON] = "Button/switch"

local DEVICE_ABILITY_TEMPERATURE     =     "02"
device_ability[DEVICE_ABILITY_TEMPERATURE] = "Temperature sensor"

local DEVICE_ABILITY_HUMIDITY        =     "03"
device_ability[DEVICE_ABILITY_HUMIDITY] = "Humidity sensor"

local DEVICE_ABILITY_PRESSURE        =     "04"
device_ability[DEVICE_ABILITY_PRESSURE] = "Pressure sensor"

local DEVICE_ABILITY_LIGHT_SENSOR    =     "05"
device_ability[DEVICE_ABILITY_LIGHT_SENSOR] = "Light sensor"

local DEVICE_ABILITY_NOISE_SENSOR    =     "06"
device_ability[DEVICE_ABILITY_NOISE_SENSOR] = "Noise sensor"

local DEVICE_ABILITY_MOTION_SENSOR   =     "07"
device_ability[DEVICE_ABILITY_MOTION_SENSOR] = "Motion sensor"

local DEVICE_ABILITY_RESERVED1       =     "08"

local DEVICE_ABILITY_C02_SENSOR      =     "09"
device_ability[DEVICE_ABILITY_C02_SENSOR] = "CO2 sensor"

local DEVICE_ABILITY_CO_SENSOR       =     "0A"
device_ability[DEVICE_ABILITY_CO_SENSOR] = "CO sensor"

local DEVICE_ABILITY_GAS_SENSOR      =     "0B"
device_ability[DEVICE_ABILITY_GAS_SENSOR] = "GAS sensor"

local DEVICE_ABILITY_POWER_METER     =     "0C"
device_ability[DEVICE_ABILITY_POWER_METER] = "Power/voltage meter"

local DEVICE_ABILITY_RADIATION_METER       =     "0D"
device_ability[DEVICE_ABILITY_RADIATION_METER] = "Radiation meter"

local DEVICE_ABILITY_RESERVED3       =     "0E"
local DEVICE_ABILITY_RESERVED4       =     "0F"
local DEVICE_ABILITY_RESERVED5       =     "10"

local DEVICE_ABILITY_RELAY           =     "11"
device_ability[DEVICE_ABILITY_RELAY] = "Relay"

local DEVICE_ABILITY_DIMMER          =     "12"
device_ability[DEVICE_ABILITY_DIMMER] = "Dimmer"

local DEVICE_ABILITY_RESERVED6       =     "13"
local DEVICE_ABILITY_RESERVED7       =     "14"
local DEVICE_ABILITY_RESERVED8       =     "15"
local DEVICE_ABILITY_RESERVED9       =     "16"
local DEVICE_ABILITY_RESERVED10      =     "17"

local DEVICE_ABILITY_LED             =     "18"
device_ability[DEVICE_ABILITY_LED] = "LED indicator"

--/*---------------------------------------------------------------------------*/--

local device_button_events = {}

local DEVICE_ABILITY_BUTTON_EVENT_CLICK        =   "01"
device_button_events[DEVICE_ABILITY_BUTTON_EVENT_CLICK] = "click"

local DEVICE_ABILITY_BUTTON_EVENT_LONG_CLICK   =   "02"
device_button_events[DEVICE_ABILITY_BUTTON_EVENT_LONG_CLICK] = "longclick"

local DEVICE_ABILITY_BUTTON_EVENT_ON           =   "03"
device_button_events[DEVICE_ABILITY_BUTTON_EVENT_ON] = "on"

local DEVICE_ABILITY_BUTTON_EVENT_OFF          =   "04"
device_button_events[DEVICE_ABILITY_BUTTON_EVENT_OFF] = "off"

--/*---------------------------------------------------------------------------*/--

local device_motionsensor_events = {}

local DEVICE_ABILITY_MOTION_SENSOR_EVENT_MOTION        =   "01"
device_motionsensor_events[DEVICE_ABILITY_MOTION_SENSOR_EVENT_MOTION] = "motion"

local DEVICE_ABILITY_MOTION_SENSOR_EVENT_NO_MOTION   =   "02"
device_motionsensor_events[DEVICE_ABILITY_MOTION_SENSOR_EVENT_NO_MOTION] = "nomotion"

--/*---------------------------------------------------------------------------*/--

local device_relay_commands = {}

local DEVICE_ABILITY_RELAY_COMMAND_ON        =   "01"
device_relay_commands[DEVICE_ABILITY_RELAY_COMMAND_ON] = "on"

local DEVICE_ABILITY_RELAY_COMMAND_OFF   =   "00"
device_relay_commands[DEVICE_ABILITY_RELAY_COMMAND_OFF] = "off"

local DEVICE_ABILITY_RELAY_COMMAND_TOGGLE   =   "80"
device_relay_commands[DEVICE_ABILITY_RELAY_COMMAND_TOGGLE] = "toggle"

--/*---------------------------------------------------------------------------*/--

local LOCAL_ROOT_COMMAND_REBOOT        =   "00"
local LOCAL_ROOT_COMMAND_BOOTLOADER_ACTIVATE    =   "01"
local LOCAL_ROOT_COMMAND_TIME_SET    =   "02"

--/*---------------------------------------------------------------------------*/--

local device_sleep_type = {}

local DEVICE_SLEEP_TYPE_NORMAL             =           "01"
device_sleep_type[DEVICE_SLEEP_TYPE_NORMAL] = "Non-sleep"

local DEVICE_SLEEP_TYPE_LEAF               =           "02"
device_sleep_type[DEVICE_SLEEP_TYPE_LEAF] = "Leaf mode"

--/*---------------------------------------------------------------------------*/--

local device_message_type = {}

local DEVICE_MESSAGE_HIGH_TEMPERATYRE             					=           "01"
device_message_type[DEVICE_MESSAGE_HIGH_TEMPERATYRE] 			= "Warning: high temperature"

local DEVICE_MESSAGE_LOW_VOLTAGE               			            =           "02"
device_message_type[DEVICE_MESSAGE_LOW_VOLTAGE] 	            = "Warning: low voltage"

local DEVICE_MESSAGE_HIGH_CURRENT               			         	=           "03"
device_message_type[DEVICE_MESSAGE_HIGH_CURRENT] 		     	= "Warning: high current"

local DEVICE_MESSAGE_LOW_POWER               			                =           "04"
device_message_type[DEVICE_MESSAGE_LOW_POWER] 	             	= "Warning: low power"

local DEVICE_MESSAGE_ERROR_ON_RELAY               		      	    =           "05"
device_message_type[DEVICE_MESSAGE_ERROR_ON_RELAY] 	         	= "Warning: error on relay"

local DEVICE_MESSAGE_ERROR_OFF_RELAY               		      	    =           "06"
device_message_type[DEVICE_MESSAGE_ERROR_OFF_RELAY] 	     	= "Warning: error off relay"

local DEVICE_MESSAGE_OTA_SPI_NOTACTIVE             					=           "07"
device_message_type[DEVICE_MESSAGE_OTA_SPI_NOTACTIVE] 			= "OTA: External flash not active"

local DEVICE_MESSAGE_OTA_NOT_DELIVERED_CHUNK               			=           "08"
device_message_type[DEVICE_MESSAGE_OTA_NOT_DELIVERED_CHUNK] 	= "OTA: Chunk not delivered"

local DEVICE_MESSAGE_OTA_NONCORRECT_CRC               				=           "09"
device_message_type[DEVICE_MESSAGE_OTA_NONCORRECT_CRC] 			= "OTA: Non-correct image CRC"

local DEVICE_MESSAGE_OTA_BAD_GOLDEN_IMAGE               			    =           "0A"
device_message_type[DEVICE_MESSAGE_OTA_BAD_GOLDEN_IMAGE] 		= "OTA: Bad golden image"

local DEVICE_MESSAGE_OTA_SPI_ERASE_IN_PROGRESS               		    =           "0B"
device_message_type[DEVICE_MESSAGE_OTA_SPI_ERASE_IN_PROGRESS] 	= "OTA: SPI flash erase in progress"

local DEVICE_MESSAGE_OTA_UPDATE_SUCCESS               		        =           "0C"
device_message_type[DEVICE_MESSAGE_OTA_UPDATE_SUCCESS] 	        = "OTA: Update success"

local DEVICE_MESSAGE_OTA_NONCORRECT_UUID               		        =           "0D"
device_message_type[DEVICE_MESSAGE_OTA_NONCORRECT_UUID] 	        = "OTA: Non-correct firmware UUID"

local DEVICE_MESSAGE_TIMESYNC_STATUS                                 =         "0E"
device_message_type[DEVICE_MESSAGE_TIMESYNC_STATUS] 	        = "TIMESYNC: Time synced"

--/*---------------------------------------------------------------------------*/--

local DATA_TYPE_FIRMWARE_COMMAND_NEW_FW              =         "01" --Сообщение о наличии новой прошивки
local DATA_TYPE_FIRMWARE_COMMAND_chunk_REQ           =         "02" --Запрос пакета с частью прошивки ???????

local PROTOCOL_VERSION_V1            =     "01"
local DEVICE_VERSION_V1              =     "01"

local UART_PROTOCOL_VERSION_V1       =     "01"
local UART_PROTOCOL_VERSION_V2       =     "02"
local UART_PROTOCOL_VERSION_V3       =     "03"
local UDUP_V4_PROTOCOL_VERSION       =     "04"

local UDUP_PV4_MQ = "16"
local UART_FF_DATA = "FF"

local VOLTAGE_PRESCALER = 16

--/*---------------------------------------------------------------------------*/--

local DATA_TYPE_JOIN                            =              "01" --Запрос на включение в сеть
local DATA_TYPE_SENSOR_DATA                     =              "02" --Данные с датчиков устройства
local DATA_TYPE_CONFIRM                 	      =              "03" --Подтверждение запроса на включение в сеть
local DATA_TYPE_PONG                            =              "04" --Подтверждение доставки пакета
local DATA_TYPE_COMMAND                         =              "05" --Команды возможностям устройства
local DATA_TYPE_STATUS                          =              "06" --Пакет со статусными данными
local DATA_TYPE_GET_STATUS                      =              "07" --Запрос статуса(не реализовано)
local DATA_TYPE_SETTINGS                        =              "08" --Команда настройки параметров
local DATA_TYPE_MESSAGE                         =              "09" --Сообщения
local DATA_TYPE_SET_TIME                        =              "0A" --Команда установки времени(не реализовано)
local DATA_TYPE_SET_SCHEDULE                    =              "0B" --Команда установки расписания(не реализовано)
local DATA_TYPE_FIRMWARE                        =              "0C" --Данные для OTA
local DATA_TYPE_UART                            =              "0D" --Команда с данными UART
local DATA_TYPE_FIRMWARE_CMD                    =              "0E" --Команды OTA
local DATA_TYPE_LOCAL_CMD                       =              "0F" --Локальные команды для координатора

--/*---------------------------------------------------------------------------*/--

local UDUP_V4_COMMAND_TYPE_NET_PACKET           =              "00"
local UDUP_V4_COMMAND_TYPE_REBOOT               =              "01"
local UDUP_V4_COMMAND_TYPE_BOOTLOADER_ACTIVATE  =              "02"
local UDUP_V4_COMMAND_TYPE_ROOT_TIME_SET        =              "03"
local UDUP_V4_COMMAND_TYPE_BINARY_CR_MODE       =              "04"
local UDUP_V4_COMMAND_TYPE_ASCII_CR_MODE        =              "05"

--/*---------------------------------------------------------------------------*/--

local port_name = "/dev/tty.usbserial-14301"
local main_cycle_permit = 1

local arg = arg
local p

--/*---------------------------------------------------------------------------*/--

function string.fromhex(str)
   str = string.gsub(str, " ", "")
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

local function colors(color)
	if (color == "red") then
		io.write(string.char(27,91,51,49,109))
	elseif (color == "none") then
		io.write(string.char(27,91,48,109))
	end
end

--/*---------------------------------------------------------------------------*/--

local raw_print = print

local function print(data)
	io.write(data or "")
	io.write("\n")
	io.flush()
end

local function print_n(data)
	io.write(data or "")
	io.flush()
end

local function print_red(data)
	colors("red")
	print(data)
	colors("none")
end

--/*---------------------------------------------------------------------------*/--

local function crc16_ansi_calc(s)
   assert(type(s) == 'string')
   local crc = 0x0000
   for i = 1, #s do
       local c = s:byte(i)
       crc = bit.bxor(crc, c)
       for j = 1, 8 do
           local k = bit.band(crc, 1)
           crc = bit.rshift(crc, 1)
           if k ~= 0 then
               crc = bit.bxor(crc, 0xA001)
           end
       end
   end
   return crc
end

--/*---------------------------------------------------------------------------*/--



local function ipv6_adress_parse(ipv6_adress)
	local adress_capturing = "(%w%w)(%w%w):(%w%w)(%w%w):(%w%w)(%w%w):(%w%w)(%w%w):(%w%w)(%w%w):(%w%w)(%w%w):(%w%w)(%w%w):(%w%w)(%w%w)"
	local _, end_1
	local a = {}

	_, end_1, _, _, _, _, _, _, _, _, a[1],a[2],a[3],a[4],a[5],a[6],a[7],a[8]  = string.find(ipv6_adress, adress_capturing)
	if (end_1 ~= nil) then
		return a
	else
		print("\nIV6P: Adress parse error "..ipv6_adress)
		return nil
	end
end

--/*---------------------------------------------------------------------------*/--

local function data_cut(all_data, segment_len)
  local max_len = segment_len
  local data_len
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

local function udup_v4_packet(packet_type, payload, address)
	local uart_packet_size = #payload
	local max_packet_size = 1500

	if (#payload > max_packet_size) then
		print("Too big packet size")
		return
	end

	local uart_packet_size_hex = bindechex.Dec2Hex(uart_packet_size)
   while (#uart_packet_size_hex < 16/8*2) do
      uart_packet_size_hex = "0"..uart_packet_size_hex
   end
	local uart_packet_size_hex_b1 = string.sub(uart_packet_size_hex, 1, 2)
	local uart_packet_size_hex_b2 = string.sub(uart_packet_size_hex, 3, 4)

	local preamble = UDUP_PV4_MQ:fromhex()..
                  UDUP_V4_PROTOCOL_VERSION:fromhex()..
                  packet_type:fromhex()..
                  address[1]:fromhex()..
                  address[2]:fromhex()..
                  address[3]:fromhex()..
                  address[4]:fromhex()..
                  address[5]:fromhex()..
                  address[6]:fromhex()..
                  address[7]:fromhex()..
                  address[8]:fromhex()..
						uart_packet_size_hex_b1:fromhex()..
						uart_packet_size_hex_b2:fromhex()

   print("Send packet:\nPreamble: "..preamble:tohex())

   payload = preamble..payload

   local payload_crc16 = crc16_ansi_calc(payload)
   local payload_crc16_hex = bindechex.Dec2Hex(payload_crc16)
   while (#payload_crc16_hex < 16/8*2) do
      payload_crc16_hex = "0"..payload_crc16_hex
   end
	local payload_crc16_hex_b1 = string.sub(payload_crc16_hex, 1, 2)
   local payload_crc16_hex_b2 = string.sub(payload_crc16_hex, 3, 4)

   payload = payload..payload_crc16_hex_b1:fromhex()
   payload = payload..payload_crc16_hex_b2:fromhex()


   print("Payload: "..payload:tohex())

   print("Packet all size: "..(#payload+#preamble)..", preamble: "..(#preamble)..", payload(non crc included): "..(uart_packet_size)..", hex(b1, b2): "..uart_packet_size_hex_b1.." "..uart_packet_size_hex_b2.."\n")

   --[[
	local table_segments = data_cut(payload, 25)
	for i = 1, #table_segments do
		p:write(table_segments[i])
		socket.sleep(0.006)
   end
   ]]
   p:write(payload)
   socket.sleep(0.01)
end

--/*---------------------------------------------------------------------------*/--

local function udup_v4_short_packet(packet_type)

	local preamble = UDUP_PV4_MQ:fromhex()..
                  UDUP_V4_PROTOCOL_VERSION:fromhex()..
                  packet_type:fromhex()

   local payload = preamble
   local payload_crc16 = crc16_ansi_calc(payload)

   local payload_crc16_hex = bindechex.Dec2Hex(payload_crc16)
   while (#payload_crc16_hex < 16/8*2) do
      payload_crc16_hex = "0"..payload_crc16_hex
   end
	local payload_crc16_hex_b1 = string.sub(payload_crc16_hex, 1, 2)
	local payload_crc16_hex_b2 = string.sub(payload_crc16_hex, 3, 4)
   payload = payload..payload_crc16_hex_b1:fromhex()
   payload = payload..payload_crc16_hex_b2:fromhex()
   print(payload_crc16_hex)

   --[[
	local table_segments = data_cut(payload, 25)
	for i = 1, #table_segments do
		p:write(table_segments[i])
		socket.sleep(0.006)
   end
   ]]
   p:write(payload)
   socket.sleep(0.01)
end

--/*---------------------------------------------------------------------------*/--



local function send_uart_data_0()
   local adress = ipv6_adress_parse("fd00:0000:0000:0000:0212:4b00:0939:300b")

   local bin_data = ""
   bin_data = bin_data..PROTOCOL_VERSION_V1:fromhex()
	bin_data = bin_data..DEVICE_VERSION_V1:fromhex()
	bin_data = bin_data..DATA_TYPE_COMMAND:fromhex()
	bin_data = bin_data..DEVICE_ABILITY_RELAY:fromhex()
	bin_data = bin_data..("01"):fromhex()
	bin_data = bin_data..("00"):fromhex()
   udup_v4_packet(UDUP_V4_COMMAND_TYPE_NET_PACKET, bin_data, adress)
end

local function send_uart_data_1()
   local adress = ipv6_adress_parse("fd00:0000:0000:0000:0212:4b00:0939:300b")

   local bin_data = ""
   bin_data = bin_data..PROTOCOL_VERSION_V1:fromhex()
	bin_data = bin_data..DEVICE_VERSION_V1:fromhex()
	bin_data = bin_data..DATA_TYPE_COMMAND:fromhex()
	bin_data = bin_data..DEVICE_ABILITY_RELAY:fromhex()
	bin_data = bin_data..("01"):fromhex()
	bin_data = bin_data..("01"):fromhex()
   udup_v4_packet(UDUP_V4_COMMAND_TYPE_NET_PACKET, bin_data, adress)
end


--/*---------------------------------------------------------------------------*/--


local function add_byte_to_buffer(buffer, byte)
	table.insert(buffer, byte)
	while (#buffer > 95) do
		table.remove(buffer, 1)
	end
end

local function clean_buffer(buffer)
	for i = 1, #buffer do
		table.insert(buffer, 0)
	end
end

local function get_buffer(buffer)
	return table.concat(buffer)
end

--/*---------------------------------------------------------------------------*/--

local function port_monitor()
   while 1 do
		local _, data_read = p:read(1, 200)
		if (data_read ~= nil) then
			io.write(data_read)
			io.flush()
		end
	end
end


--/*---------------------------------------------------------------------------*/--

local function main_cycle(limit, adresses_print_mode)
	local _, data_read, packet, message
	local end_time, now_time
	local main_cycle_limit_reached = 0
	main_cycle_permit = 1
	local buffer = {}
	if (limit ~= nil) then
		now_time = socket.gettime()*1000
		end_time = now_time+(limit*1000)
	end

	while (main_cycle_permit == 1 and main_cycle_limit_reached == 0) do
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
            if (message == "UDM: Time sync needed\n") then
               root_time_sync();
            else
               led("on")
               print(message)
               led("off")
            end
            clean_buffer(buffer)
			end
		end
		if (limit ~= nil) then
			now_time = socket.gettime()*1000
			if (now_time > end_time) then
				main_cycle_limit_reached = 1
			end
		end

		local exit_flag = linda:get("exit_flag")
		if (exit_flag ~= nil) then
			main_cycle_permit = 0
			linda:set("exit_flag", nil)
		end

	end
	return main_cycle_limit_reached
end


--/*---------------------------------------------------------------------------*/--


local rs232_error
rs232_error, p = rs232.open(port_name)
if rs232_error ~= rs232.RS232_ERR_NOERROR then
	print(string.format("can't open serial port '%s', error: '%s'\n",
			port_name, rs232.error_tostring(rs232_error)))
	return
end

assert(p:set_baud_rate(rs232.RS232_BAUD_115200) == rs232.RS232_ERR_NOERROR)
assert(p:set_data_bits(rs232.RS232_DATA_8) == rs232.RS232_ERR_NOERROR)
assert(p:set_parity(rs232.RS232_PARITY_NONE) == rs232.RS232_ERR_NOERROR)
assert(p:set_stop_bits(rs232.RS232_STOP_1) == rs232.RS232_ERR_NOERROR)
assert(p:set_flow_control(rs232.RS232_FLOW_OFF) == rs232.RS232_ERR_NOERROR)

--/*---------------------------------------------------------------------------*/--
--send_uart_data_1()
--send_uart_data_0()
--port_monitor()
udup_v4_short_packet(UDUP_V4_COMMAND_TYPE_ASCII_CR_MODE)

--[[

local now_epoch = os.time()

local now_epoch_hex = bindechex.Dec2Hex(now_epoch)
while (#now_epoch_hex < 32/8*2) do
   now_epoch_hex = "0"..now_epoch_hex
end
local epoch_hex_bytes = {}
epoch_hex_bytes[1] = string.sub(now_epoch_hex, 1, 2)
epoch_hex_bytes[2] = string.sub(now_epoch_hex, 3, 4)
epoch_hex_bytes[3] = string.sub(now_epoch_hex, 5, 6)
epoch_hex_bytes[4] = string.sub(now_epoch_hex, 7, 8)

local address = {}
address[1] = "00"
address[2] = "00"
address[3] = "00"
address[4] = "00"
address[5] = "00"
address[6] = "00"
address[7] = "00"
address[8] = "00"

local epoch_bin = epoch_hex_bytes[1]:fromhex()..epoch_hex_bytes[2]:fromhex()..epoch_hex_bytes[3]:fromhex()..epoch_hex_bytes[4]:fromhex()

udup_v4_packet(UDUP_V4_COMMAND_TYPE_ROOT_TIME_SET, epoch_bin, address)
]]
