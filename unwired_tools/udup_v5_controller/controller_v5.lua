#!/usr/bin/lua

local script_dir = (debug.getinfo(1).source:match("@?(.*/)")) or ""
package.path = package.path..';'..script_dir.."?.lua"

local socket = require("socket")

local bindechex = require("lib_bindechex")

local posix = require("posix")

local version = require("version")
local ver = version.git

local lanes = require "lanes".configure()

local linda = lanes.linda()

local ansicrc = require("lib_ansicrc")

local strings = require("lib_strings")
string.fromhex = strings.fromhex
string.tohex = strings.tohex
local print_raw = print
local print = strings.print
local print_n = strings.print_n
local print_red = strings.print_red

local cjson = require "cjson"

local mqtt = require("lib_mqtt")
local mqtt_client

local bit = require "bit"

local arg = arg

--/*-------------------------------- Дефайны и определения -------------------------------------------*/--


local DATA_TYPE_RESERVED_1                      =              "01" --Не используется
local DATA_TYPE_SENSOR_DATA                     =              "02" --Данные с датчиков устройства
local DATA_TYPE_RESERVED_2              	      =              "03" --Не используется
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
local DATA_TYPE_RESERVED_3                      =              "0F" --Не используется
local DATA_TYPE_JOIN_V5_STAGE_1                 =              "10" --Нода посылает запрос координатору
local DATA_TYPE_JOIN_V5_STAGE_2                 =              "11" --Координатор отвечает ноде


local UNWDS_GPIO_MODULE_ID = "01"
local UNWDS_4BTN_MODULE_ID = "02"
local UNWDS_OPT3001_MODULE_ID = "0F"
local UNWDS_6LOWPAN_SYSTEM_MODULE_ID = "7F"


local DATA_TYPE_MESSAGES = {}
DATA_TYPE_MESSAGES["0F"] = "Join ok"


--/*------------------------------ Системные функции ---------------------------------------------*/--

local function send_to_uart(packet)
   --print("uart packet send: ", packet)
   linda:set("uart_new_data", packet)
end

local function send_to_mqtt(text, topic)
   --print(topic, ":\n", text)
   mqtt_client:publish(topic, text)
end

local function publish_error_message(error_message)
   mqtt_client:publish("error", error_message)
   print(error_message)
end

local function net_packet_send(address, payload)
   local UDUP_V5_MAGIC_BYTE = "16"
   local UDUP_V5_PROTOCOL_VERSION = "05"
   local UDUP_V5_COMMAND_TYPE_NET_PACKET = "00"

   local payload_length_dec = #payload / 2
   local payload_length_hex_b1, payload_length_hex_b2  = bindechex.Dec2Hex_augment_2b(payload_length_dec, 4)

   local packet = ""
   packet = packet .. UDUP_V5_MAGIC_BYTE
   packet = packet .. UDUP_V5_PROTOCOL_VERSION
   packet = packet .. UDUP_V5_COMMAND_TYPE_NET_PACKET
   packet = packet .. address
   packet = packet .. payload_length_hex_b1
   packet = packet .. payload_length_hex_b2

   packet = packet .. payload

   local packet_crc_dec = ansicrc.calc_hex(packet)
   local packet_crc_hex_b1, packet_crc_hex_b2  = bindechex.Dec2Hex_augment_2b(packet_crc_dec, 4)

   packet = packet .. packet_crc_hex_b1
   packet = packet .. packet_crc_hex_b2

   print("Send packet: ", packet)
   send_to_uart(packet)
end

--/*------------------------------ Обработка разных типов приходящих системных пакетов ---------------------------------------------*/--

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
   send_to_mqtt(json_text, "devices/6lowpan/"..address_string.."/status")
end

local function message_data_processing(address_string, voltage, parent_rssi, payload)
   local json_table = {}
   json_table.data = {}
   json_table.data.message = DATA_TYPE_MESSAGES[payload[3]] or payload[3]

   json_table.status = {}
   json_table.status.devEUI = address_string
   json_table.status.rssi = parent_rssi
   json_table.status.temperature = 0
   json_table.status.battery = voltage
   json_table.status.date = os.date("!%FT%T.%uZ")
   local json_text = cjson.encode(json_table)
   send_to_mqtt(json_text, "devices/6lowpan/"..address_string.."/message")
end

local function unwds_opt3001_data_processing(address_string, voltage, parent_rssi, payload)

   local json_table = {}
   json_table.data = {}
   json_table.data.luminocity = tonumber(bindechex.Hex2Dec((payload[3] or 00)..(payload[2] or 00)))

   json_table.status = {}
   json_table.status.devEUI = address_string
   json_table.status.rssi = parent_rssi
   json_table.status.temperature = 0
   json_table.status.battery = voltage
   json_table.status.date = os.date("!%FT%T.%uZ")

   local json_text = cjson.encode(json_table)
   send_to_mqtt(json_text, "devices/6lowpan/"..address_string.."/opt3001")
end

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
   send_to_mqtt(json_text, "devices/6lowpan/"..address_string.."/4btn")
end

local function unwds_gpio_data_processing(address_string, voltage, parent_rssi, payload)
   local json_table = {}
   json_table.data = {}

   local GPIO_MODULE_ANSWER = {}
   GPIO_MODULE_ANSWER["00"] = "UMDK_GPIO_REPLY_OK_0"
	GPIO_MODULE_ANSWER["01"] = "UMDK_GPIO_REPLY_OK_1"
	GPIO_MODULE_ANSWER["02"] = "UMDK_GPIO_REPLY_OK"
	GPIO_MODULE_ANSWER["03"] = "UMDK_GPIO_REPLY_ERR_PIN"
	GPIO_MODULE_ANSWER["04"] = "UMDK_GPIO_REPLY_ERR_FORMAT"
   GPIO_MODULE_ANSWER["05"] = "UMDK_GPIO_REPLY_OK_AINAF"
   GPIO_MODULE_ANSWER["06"] = "UMDK_GPIO_REPLY_OK_ALL"

   local answer_byte = payload[2]
   json_table.data.answer = GPIO_MODULE_ANSWER[answer_byte]

   json_table.status = {}
   json_table.status.devEUI = address_string
   json_table.status.rssi = parent_rssi
   json_table.status.temperature = 0
   json_table.status.battery = voltage
   json_table.status.date = os.date("!%FT%T.%uZ")

   local json_text = cjson.encode(json_table)
   send_to_mqtt(json_text, "devices/6lowpan/"..address_string.."/gpio")
end


--/*------------------------------ Создание 6LOWPAN пакетов ---------------------------------------------*/--

local function gpio_message_create(address, message)
   local gpio_capture_pattern = "^(%a%a%a)%s(%d%d-)%s(%d)$" --"set 14 1" or "get 14"
   local _, _, gpio_cmd, gpio_num, gpio_state = string.find(message, gpio_capture_pattern)
   gpio_state = tonumber(gpio_state)
   gpio_num = tonumber(gpio_num)
   if (gpio_cmd == nil or gpio_num == nil) then
      return
   end
   if (gpio_cmd ~= "set" or gpio_cmd ~= "get") then
      publish_error_message("Non-correct gpio cmd(set or get): "..(message or ""))
      return
   end
   if (gpio_num == nil or (gpio_num < 0 or gpio_num > 50)) then
      publish_error_message("Non-correct gpio num(0-50): "..(message or ""))
   end
   if (gpio_cmd ~= "set" and (gpio_state == nil or gpio_state ~= 0 or gpio_state ~= 1)) then
      publish_error_message("Non-correct gpio state(0 or 1): "..(message or ""))
      return
   end

   print("MQTT gpio parser: gpio_cmd: ", (gpio_cmd or "nil"), ", gpio_num: ", (gpio_num or "nil"), ", gpio_state: ", (gpio_state or "nil"), ", address: ", (address or "nil"))

   local payload = ""
   local command_gpio_byte
   local int_command

   if (gpio_cmd ~= "get" and gpio_cmd ~= "set") then
      return
   end

   if (gpio_state ~= 0 and gpio_state ~= 1) then
      return
   end

   if (gpio_state == 0) then
      int_command = 1
   elseif (gpio_state == 1) then
      int_command = 2
   end

   if (gpio_cmd == "set") then
      gpio_num = bit.tobit(gpio_num)
      int_command = bit.tobit(int_command)

      local int_command_shift = bit.lshift(int_command, 6)
      command_gpio_byte = bit.bor(int_command_shift, gpio_num)
   elseif (gpio_cmd == "get") then
      --command_gpio_byte =
      publish_error_message("GPIO get not supported")
   end

   payload = payload .. UNWDS_GPIO_MODULE_ID
   payload = payload .. bindechex.Dec2Hex_augment(command_gpio_byte)

   net_packet_send(address, payload)
end

local function opt3001_message_create(address, message)
   local opt3001_capture_pattern = "([_%a]+)%s(%d*)" --"set_period 1" or "poll"
   local _, _, opt3001_cmd, opt3001_period = string.find(message, opt3001_capture_pattern)

   opt3001_period = tonumber(opt3001_period)

   if (opt3001_cmd == nil) then
      return
   end
   if (opt3001_cmd ~= "set_period" and opt3001_cmd ~= "poll") then
      publish_error_message("Non-correct opt3001 command(poll or set_period): "..(message or ""))
      return
   end
   if (opt3001_cmd == "set_period" and opt3001_period == nil) then
      publish_error_message("Non-correct opt3001 period: "..(message or ""))
      return
   end
   if (opt3001_cmd == "set_period" and opt3001_period > 255) then
      publish_error_message("Non-correct opt3001 period(>255): "..(message or ""))
      return
   end
   print("MQTT opt3001 parser: opt3001_cmd: ", (opt3001_cmd or "nil"), ", opt3001_period: ", (opt3001_period or "nil"), ", address: ", (address or "nil"))

   local payload = ""

   local UMDK_OPT3001_CMD_SET_PERIOD = "00"
   local UMDK_OPT3001_CMD_POLL = "01"

   payload = payload .. UNWDS_OPT3001_MODULE_ID

   if (opt3001_cmd == "set_period") then
      opt3001_period = tonumber(opt3001_period) or 0
      if opt3001_period > 255 then opt3001_period = 255 end
      if opt3001_period < 0 then opt3001_period = 1 end
      payload = payload .. UMDK_OPT3001_CMD_SET_PERIOD
      payload = payload .. bindechex.Dec2Hex_augment(opt3001_period)
   elseif (opt3001_cmd == "poll") then
      payload = payload .. UMDK_OPT3001_CMD_POLL
   end

   net_packet_send(address, payload)
end

--/*------------------------------ Обработка MQTT -> 6LOWPAN и 6LOWPAN -> MQTT пакетов ---------------------------------------------*/--

local function uart_packet_parse(packet)
   local packet_capture_pattern = "160500(%w%w%w%w%w%w%w%w%w%w%w%w%w%w%w%w)(%w%w)(%w%w)(%w%w%w%w)(.+)(%w%w%w%w)"
   local _, _, adress_raw, voltage_raw, parent_rssi_raw, payload_length_raw, payload_raw, crc_raw = string.find(packet, packet_capture_pattern)
   print("\nRaw packet: ", adress_raw, " ", voltage_raw,  " ", parent_rssi_raw, " ", payload_length_raw, " ", payload_raw, " ", crc_raw)
   if (adress_raw ~= nil and payload_length_raw ~= nil and payload_raw ~= nil and crc_raw ~= nil and voltage_raw ~= nil and parent_rssi_raw ~= nil) then
      local adress_capture_pattern = "(%w%w%w%w)(%w%w%w%w)(%w%w%w%w)(%w%w%w%w)"
      local payload_length_capture_pattern, crc_capture_pattern, payload_byte_capture_pattern = "(%w%w)(%w%w)", "(%w%w)(%w%w)", "(%w%w)"

		local _, i = nil, 1
		local a, payload, payload_length, crc = {}, {}, {}, {}

		_, _, a[1], a[2], a[3], a[4]  = string.find(adress_raw, adress_capture_pattern)
      _, _, payload_length[1], payload_length[2] = string.find(payload_length_raw, payload_length_capture_pattern)
      _, _, crc[1], crc[2] = string.find(crc_raw, crc_capture_pattern)

      for w in string.gmatch(payload_raw, payload_byte_capture_pattern) do
         payload[i] = w
         i = i + 1
      end

      local parent_rssi = (tonumber(bindechex.Hex2Dec(parent_rssi_raw or 00)))-200
      local voltage = (((tonumber(bindechex.Hex2Dec(voltage_raw or 00)))*50)+2000)
      local payload_length_dec = tonumber(bindechex.Hex2Dec((payload_length[1] or 00)..(payload_length[2] or 00)))
      local crc_dec = tonumber(bindechex.Hex2Dec((crc[1] or 00)..(crc[2] or 00)))
      local address_string = a[1]..a[2]..a[3]..a[4]
      local bin_packet = strings.hex_to_bin_string(packet)
      local bin_packet_no_crc = string.sub(bin_packet, 0, #bin_packet-2) --Cut crc bytes(2b)
      local crc_calculated = ansicrc.calc(bin_packet_no_crc)

      if (payload_length_dec ~= #payload) then
         print("Non-corrent payload length: ", payload_length_dec, "/", #payload)
         return
      end
      if (crc_dec ~= crc_calculated) then
         print("Non-corrent crc packet: ", bindechex.Dec2Hex(crc_dec), "/", bindechex.Dec2Hex(crc_calculated))
         return
      end

      local module_id = payload[1]
      if (module_id == UNWDS_6LOWPAN_SYSTEM_MODULE_ID) then
         local packet_type = payload[2]
         if (packet_type == DATA_TYPE_JOIN_V5_STAGE_1) then
            print("Not parse join stage 1 packet(it is normal)")
            --join_data_processing(address_string, voltage, parent_rssi, payload)
         elseif (packet_type == DATA_TYPE_STATUS) then
            status_data_processing(address_string, voltage, parent_rssi, payload)
         elseif (packet_type == DATA_TYPE_MESSAGE) then
            message_data_processing(address_string, voltage, parent_rssi, payload)
         else
            print("Non-corrent packet type: ", packet_type, "\t", payload_raw)
         end
      elseif (module_id == UNWDS_OPT3001_MODULE_ID) then
         unwds_opt3001_data_processing(address_string, voltage, parent_rssi, payload)
      elseif (module_id == UNWDS_4BTN_MODULE_ID) then
         unwds_4btn_data_processing(address_string, voltage, parent_rssi, payload)
      elseif (module_id == UNWDS_GPIO_MODULE_ID) then
         unwds_gpio_data_processing(address_string, voltage, parent_rssi, payload)
      else
         print("Non-corrent module id: ", module_id, "\t", payload_raw)
      end


	else
		print("Not parse packet: "..packet)
	end
end

local function mqtt_message_parse(topic, message)
   local json_capture_pattern = "(%b{})"
   local _, _, json = string.find(message, json_capture_pattern)
   if (json ~= nil) then
      return
   end

   local topic_capture_pattern = "devices/(.+)/(.+)/(.+)"
   local _, _, main_target, address, sub_target = string.find(topic, topic_capture_pattern)

   print("MQTT->6LP: ", (topic or "nil"), ": ", (message or "nil"))

   if (main_target ~= "6lowpan") then
      return
   end

   if (address == "6lowpan_uart")  then
      send_to_uart(message)
      return
   end

   --print("MQTT->6LP: main_target: ", (main_target or "nil"), ", address: ", (address or "nil"), ", sub_target: ", (sub_target or "nil"))

   if (sub_target == "gpio") then
      gpio_message_create(address, message)
   elseif (sub_target == "opt3001") then
      opt3001_message_create(address, message)
   end


end

--/*---------------------------------------------------------------------------*/--

local function main_cycle()
   mqtt_client = mqtt.client.create("localhost", 1883, mqtt_message_parse)
   local mqtt_err = mqtt_client:connect("lua_parser")
   if (mqtt_err ~= nil) then
      print(mqtt_err)
      os.exit(0)
   end
   print("MQTT: Connected")

   mqtt_client:subscribe({"devices/6lowpan/#"})
   print("MQTT: Subscribed to devices/6lowpan/#")

   send_to_mqtt("6lowpan<->MQTT LUA parser start", "/devices/6lowpan/system")

	while true do
      socket.sleep(0.01)
      mqtt_client:handler()
		local uart_packet = linda:get("uart_packet")
      if (uart_packet ~= nil) then
         linda:set("uart_packet", nil)
         --print(uart_packet)
         --mqtt_client:publish("devices/6lowpan/uart/", uart_packet)
         uart_packet_parse(uart_packet)
		end
	end
end

--/*---------------------------------------------------------------------------*/--

local function lanes_uart_parser()

   local socket = require("socket")

   local rs232 = require("luars232")
   local port_name = "/dev/ttyATH0"

   local buffers = require("lib_buffers")
   local buffer = buffers.create_buffer(150)

   local strings = require("lib_strings")

   local e, p = rs232.open(port_name)
   if (e ~= rs232.RS232_ERR_NOERROR) then
      print(string.format("can't open serial port '%s', error: '%s'\n", port_name, rs232.error_tostring(e)))
      return
   end

   p:set_baud_rate(rs232.RS232_BAUD_115200)
   p:set_data_bits(rs232.RS232_DATA_8)
   p:set_parity(rs232.RS232_PARITY_NONE)
   p:set_stop_bits(rs232.RS232_STOP_1)
   p:set_flow_control(rs232.RS232_FLOW_OFF)

   local _, data_read, packet, message
   local buffer_state, uart_new_data

   print("UART: Parser active")

   while true do

      uart_new_data = linda:get("uart_new_data")
      if (uart_new_data ~= nil) then
         local table_segments = strings.cut(uart_new_data, 30)
         for i = 1, #table_segments do
            p:write(table_segments[i])
            socket.sleep(0.006)
         end
         p:write("\n")
         linda:set("uart_new_data", nil)
      end

      _, data_read = p:read(1, 1)
      if (data_read ~= nil) then
         buffers.add_byte_to_buffer(buffer, data_read)
         buffer_state = buffers.get_buffer(buffer)
         --print(buffer_state)
         if (data_read == "\n") then
            _, _, packet = string.find(buffer_state, "(1605.+\n)")
            if (packet ~= nil) then
               linda:set("uart_packet", packet)
               --print(buffer_state)
               buffers.clean_buffer(buffer)
            end
            _, _, message = string.find(buffer_state, "(UDM:.+)\n")
            if (message ~= nil) then
               print("\n"..message.."\n")
               --linda:set("uart_packet", packet)
               buffers.clean_buffer(buffer)
            end
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

