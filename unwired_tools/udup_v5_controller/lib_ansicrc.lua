local ansicrc = {}
local strings = require("lib_strings")
string.fromhex = strings.fromhex

function ansicrc.calc(s)
   local bit = require "bit"
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

function ansicrc.calc_hex(s)
   local bin_s = ""

   for w in string.gmatch(s, "(%w%w)") do
      bin_s = bin_s .. w:fromhex()
   end
   return ansicrc.calc(bin_s)
end

--/*---------------------------------------------------------------------------*/--

return ansicrc
