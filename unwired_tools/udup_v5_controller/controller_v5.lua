#!/usr/bin/lua

local script_dir = (debug.getinfo(1).source:match("@?(.*/)")) or ""
package.path = package.path..';'..script_dir.."?.lua"

local socket = require("socket")

local bindechex = require("bindechex")

local posix = require("posix")

local version = require("version")
local ver = version.git

local lanes = require "lanes".configure()

local linda = lanes.linda()

local newprint = require("lib_newprint")
local raw_print = print
local print = newprint.print
local print_n = newprint.print_n
local print_red = newprint.print_red

local ansicrc = require("lib_ansicrc")

local strings = require("lib_strings")
string.fromhex = strings.fromhex
string.tohex = strings.tohex

local cjson = require "cjson"

local mqtt = require("mqtt_library")
local mqtt_client = mqtt.client.create("localhost", 1883, mqtt_callback)

local bit = require "bit"

local arg = arg

--/*-------------------------------- Дефайны и определения -------------------------------------------*/--


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


local PAYLOAD_OFFSET_MODULE_ID = 1

local UNWDS_4BTN_MODULE_ID = 2
local UNWDS_OPT3001_MODULE_ID = 15

local UNWDS_CONFIG_MODULE_ID = 126
local PAYLOAD_CONFIG_MODULE_ID_OFFSET_PACKET_TYPE = 2

--/*---------------------------------------------------------------------------*/--

local function get_module_id(payload)
   return bindechex.Hex2Dec(payload[PAYLOAD_OFFSET_MODULE_ID])
end

--/*---------------------------------------------------------------------------*/--

local function get_module_config_packet_type(payload)
   return bindechex.Hex2Dec(payload[PAYLOAD_CONFIG_MODULE_ID_OFFSET_PACKET_TYPE])
end

--/*---------------------------------------------------------------------------*/--

local function send_to_mqtt(json_text, topic)
   --print(topic, ":\n", json_text)
   mqtt_client:publish(topic, json_text)
end

--/*------------------------------ Обработка разных типов системных пакетов ---------------------------------------------*/--

local function join_data_processing(address, voltage, rssi, payload)
   local json_table = {}
   json_table.data = {}
   json_table.data.device_group = device_group[current_device_group] or current_device_group
   json_table.data.sleep_type = device_sleep_type[current_sleep_type] or current_sleep_type

   json_table.status = {}
   json_table.status.devEUI = address_string
   json_table.status.rssi = parent_rssi
   json_table.status.temperature = 0
   json_table.status.battery = voltage
   json_table.status.date = os.date("!%FT%T.%uZ")
   local json_text = cjson.encode(json_table)
   send_to_mqtt(json_text, "/devices/6lowpan/"..address_string.."/join")
end

--/*---------------------------------------------------------------------------*/--

local function status_data_processing(address_string, voltage, parent_rssi, payload)
   local json_table = {}
   json_table.data = {}
   json_table.data.parent_address = payload[3]..payload[4]..payload[5]..payload[6]..payload[7]..payload[8]..payload[9]..payload[10]
   json_table.data.sw_version = bindechex.Hex2Dec(payload[16] or 00).."."..bindechex.Hex2Dec(payload[17] or 00)
   json_table.data.uptime = tonumber(bindechex.Hex2Dec((payload[14] or 00)..(payload[13] or 00)..(payload[12] or 00)..(payload[11] or 00)))
   json_table.data.temperature = bindechex.Hex2Dec(payload[15] or 00)

   json_table.status = {}
   json_table.status.devEUI = address_string
   json_table.status.rssi = parent_rssi
   json_table.status.temperature = 0
   json_table.status.battery = voltage
   json_table.status.date = os.date("!%FT%T.%uZ")
   local json_text = cjson.encode(json_table)
   send_to_mqtt(json_text, "/devices/6lowpan/"..address_string.."/status")
end

--/*------------------------------ Обработка пакетов UNWDS ---------------------------------------------*/--

local function unwds_opt3001_data_processing(address_string, voltage, parent_rssi, payload)

   local json_table = {}
   json_table.data = {}
   json_table.data.luminocity = 0

   json_table.status = {}
   json_table.status.devEUI = address_string
   json_table.status.rssi = parent_rssi
   json_table.status.temperature = 0
   json_table.status.battery = voltage
   json_table.status.date = os.date("!%FT%T.%uZ")

   local json_text = cjson.encode(json_table)
   send_to_mqtt(json_text, "/devices/6lowpan/"..address_string.."/opt3001")
end

--/*---------------------------------------------------------------------------*/--

local function unwds_4btn_data_processing(address_string, voltage, parent_rssi, payload)

   local json_table = {}
   json_table.data = {}

   local btn_byte = payload[2]
   local dio_num_mask = bindechex.Bin2Hex("00111111")
   local press_mask = bindechex.Bin2Hex("10000000")
   local long_mask = bindechex.Bin2Hex("01000000")
   local btn_gpio = bindechex.BMAnd(btn_byte, dio_num_mask)
   json_table.data.button = bindechex.Hex2Dec(btn_gpio)

   if (tonumber(bindechex.BMAnd(btn_byte, press_mask)) ~= 0) then
      if (tonumber(bindechex.BMAnd(btn_byte, long_mask)) ~= 0) then
         json_table.data.state = "long_press"
      else
         json_table.data.state = "short_press"
      end
   end

   json_table.status = {}
   json_table.status.devEUI = address_string
   json_table.status.rssi = parent_rssi
   json_table.status.temperature = 0
   json_table.status.battery = voltage
   json_table.status.date = os.date("!%FT%T.%uZ")

   local json_text = cjson.encode(json_table)
   send_to_mqtt(json_text, "/devices/6lowpan/"..address_string.."/4btn")
end

--/*---------------------------------------------------------------------------*/--

local function packet_parse(packet)
   local packet_capture_pattern = "160500(%w%w%w%w%w%w%w%w%w%w%w%w%w%w%w%w)(%w%w)(%w%w)(%w%w%w%w)(.+)(%w%w%w%w)"
   local _, _, adress_raw, voltage_raw, parent_rssi_raw, payload_length_raw, payload_raw, crc_raw = string.find(packet, packet_capture_pattern)
   --raw_print(adress_raw, voltage_raw, parent_rssi_raw, payload_length_raw, payload_raw, crc_raw)
   if (adress_raw ~= nil and payload_length_raw ~= nil and payload_raw ~= nil and crc_raw ~= nil and voltage_raw ~= nil and parent_rssi_raw ~= nil) then
      local adress_capture_pattern = "(%w%w%w%w)(%w%w%w%w)(%w%w%w%w)(%w%w%w%w)"
      local payload_length_capture_pattern = "(%w%w)(%w%w)"
      local crc_capture_pattern = "(%w%w)(%w%w)"

		local _
		local a = {}
      local d = {}
      local payload_length = {}
      local crc = {}
      local payload = {}

		_, _, a[1], a[2], a[3], a[4]  = string.find(adress_raw, adress_capture_pattern)
      _, _, payload_length[1], payload_length[2] = string.find(payload_length_raw, payload_length_capture_pattern)
      _, _, crc[1], crc[2] = string.find(crc_raw, crc_capture_pattern)

      local i = 1
      for w in string.gmatch(payload_raw, "(%w%w)") do
         payload[i] = w
         i = i + 1
      end

      local parent_rssi = (tonumber(bindechex.Hex2Dec(parent_rssi_raw or 00)))-200
      local voltage = (((tonumber(bindechex.Hex2Dec(voltage_raw or 00)))*50)+2000)
      local payload_length_dec = tonumber(bindechex.Hex2Dec((payload_length[2] or 00)..(payload_length[1] or 00)))
      local crc_dec = tonumber(bindechex.Hex2Dec((crc[1] or 00)..(crc[2] or 00)))
      local address_string = a[1]..a[2]..a[3]..a[4]
      local bin_packet = strings.hex_to_bin_string(packet)
      local bin_packet_no_crc = string.sub(bin_packet, 0, #bin_packet-2)
      local crc_calculated = ansicrc.calc(bin_packet_no_crc)

      if (payload_length_dec ~= #payload) then
         print("Non-corrent payload length: ", payload_length_dec, "/", #payload)
         return
      end
      if (crc_dec ~= crc_calculated) then
         print("Non-corrent crc packet: ", bindechex.Dec2Hex(crc_dec), "/", bindechex.Dec2Hex(crc_calculated))
         return
      end

      if (get_module_id(payload) == UNWDS_CONFIG_MODULE_ID) then
         if (get_module_config_packet_type(payload) == DATA_TYPE_JOIN) then
            join_data_processing(address_string, voltage, parent_rssi, payload)
         elseif (get_module_config_packet_type(payload) == DATA_TYPE_STATUS) then
            status_data_processing(address_string, voltage, parent_rssi, payload)
         else
            print("Non-corrent packet type: ", get_module_config_packet_type(payload))
         end
      elseif (get_module_id(payload) == UNWDS_OPT3001_MODULE_ID) then
         unwds_opt3001_data_processing(address_string, voltage, parent_rssi, payload)
      elseif (get_module_id(payload) == UNWDS_4BTN_MODULE_ID) then
         unwds_4btn_data_processing(address_string, voltage, parent_rssi, payload)
      else
         print("Non-corrent module id: ", get_module_id(payload))
      end

--[[
      print_n("payload:")
      for i2 = 1, #payload do
         print_n(payload[i2].."  ")
      end
      print_n("\n")
]]

--[[
		if (end_1 ~= nil and end_2 ~= nil) then
			packet_processing(a, d)
		else
			print(string.format("Not parse adress(%s) or data(%s) in packet"), tostring(end_1), tostring(end_2))
			print(packet)
			print(adress)
			print(adress_capturing)
			print(raw_data)
			print(data_capturing)
      end
]]
	else
		print("Not parse packet: "..packet)
	end
end

local function uart_packet_process(packet)
   packet_parse(packet)
   mqtt_client:publish("uart/", packet)
end

local function uart_packet_send(packet)
   linda:set("uart_new_data", packet)
end


local function mqtt_callback(topic, message)
   print("MQTT: ", topic, ": ", message)
end


--/*---------------------------------------------------------------------------*/--

local function main_cycle()
	mqtt_client:connect("lua_parser")
   --mqtt_client:subscribe("#")

	while true do
		socket.sleep(0.01)
		local uart_packet = linda:get("uart_packet")
      if (uart_packet ~= nil) then
         linda:set("uart_packet", nil)
         --print(uart_packet)
         packet_parse(uart_packet)
		end
	end
end

--/*---------------------------------------------------------------------------*/--

local function lanes_uart_parser()

   local socket = require("socket")

   local rs232 = require("luars232")
   local port_name = "/dev/ttyATH0"

   local bindechex = require("bindechex")

   local buffers = require("lib_buffers")

   local strings = require("lib_strings")
   string.fromhex = strings.fromhex
   string.tohex = strings.tohex

   local newprint = require("lib_newprint")
   local raw_print = print
   local print = newprint.print
   local print_n = newprint.print_n
   local print_red = newprint.print_red


   local e, p = rs232.open(port_name)
   if e ~= rs232.RS232_ERR_NOERROR then
      print(string.format("can't open serial port '%s', error: '%s'\n",
      port_name, rs232.error_tostring(e)))
      return
   end

   p:set_baud_rate(rs232.RS232_BAUD_115200)
   p:set_data_bits(rs232.RS232_DATA_8)
   p:set_parity(rs232.RS232_PARITY_NONE)
   p:set_stop_bits(rs232.RS232_STOP_1)
   p:set_flow_control(rs232.RS232_FLOW_OFF)

   local _, data_read, packet, message
   local buffer = buffers.create_buffer()

   print("LANES thread 1: Start uart parser cycle")

   while true do
      local uart_new_data = linda:get("uart_new_data")
      if (uart_new_data ~= nil) then
         for i = 1, #uart_new_data do
            p:write(uart_new_data[i])
            --socket.sleep(0.006)
         end
         linda:set("uart_new_data", nil)
      end

      _, data_read = p:read(1, 1)
      if (data_read ~= nil) then
         buffers.add_byte_to_buffer(buffer, data_read)
         local buffer_state = buffers.get_buffer(buffer)
         _, _, packet = string.find(buffer_state, "(1605.+\n)")
         if (packet ~= nil) then
            linda:set("uart_packet", packet)
            buffers.clean_buffer(buffer)
         end
         _, _, message = string.find(buffer_state, "(UDM:.+\n)")
         if (message ~= nil) then
            print(message)
            buffers.clean_buffer(buffer)
         end
      end
   end

end

--/*---------------------------------------------------------------------------*/--

local function entry_point()
   print_red("RPL-router v5, version "..ver..", uart protocol version: 5\n")

   if (arg[1] == "main") then
      lanes.gen("*",lanes_uart_parser)()
      main_cycle()
   else
      print("Use:\trouter.lua main \t\tstart main loop(data parse/show)")
   end
end

--/*---------------------------------------------------------------------------*/--

entry_point()
--packet_parse("16050002124B000939300B1AB010000286FFFFFFFFFFFFFFFFFFFFFFFFFFFFA978")
