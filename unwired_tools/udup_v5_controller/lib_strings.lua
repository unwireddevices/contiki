local strings = {}

--/*---------------------------------------------------------------------------*/--

function strings.color(color)
	if (color == "red") then
		io.write(string.char(27,91,51,49,109))
	elseif (color == "none") then
		io.write(string.char(27,91,48,109))
	end
end

--/*---------------------------------------------------------------------------*/--

function strings.print(data, ...)
	io.write(data, ...)
	io.write("\n")
	io.flush()
end

--/*---------------------------------------------------------------------------*/--

function strings.print_n(data, ...)
	io.write(data, ...)
	io.flush()
end

function strings.print_red(data, ...)
	strings.color("red")
	print(data, ...)
	strings.color("none")
end


--/*---------------------------------------------------------------------------*/--

function strings.fromhex(str)
   local str = string.gsub(str, " ", "")
   return (str:gsub('..', function (cc)
       return string.char(tonumber(cc, 16))
   end))
end

--/*---------------------------------------------------------------------------*/--

function strings.tohex(str)
   return (str:gsub('.', function (c)
      return string.format('%02X ', string.byte(c))
  end))
end

--/*---------------------------------------------------------------------------*/--

function strings.cut(all_data, segment_len)
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

function strings.hex_to_bin_string(hex_string)
   local bin_string = ""
   for w in string.gmatch(hex_string, "(%w%w)") do
      bin_string = bin_string..(strings.fromhex(w))
   end
   return bin_string
 end

--/*---------------------------------------------------------------------------*/--

 function strings.print_table(t)
	if (type(t) == "table") then
	    local print_r_cache={}
	    local function sub_print_r(t,indent)
	        if (print_r_cache[tostring(t)]) then
	            print(indent.."*"..tostring(t))
	        else
	            print_r_cache[tostring(t)]=true
	            if (type(t)=="table") then
	                for pos,val in pairs(t) do
	                    if (type(val)=="table") then
	                        print(indent.."["..pos.."] => "..tostring(t).." {")
	                        sub_print_r(val,indent..string.rep(" ",string.len(pos)+8))
	                        print(indent..string.rep(" ",string.len(pos)+6).."}")
	                    elseif (type(val)=="string") then
	                        print(indent.."["..pos..'] => "'..val..'"')
	                    else
	                        print(indent.."["..pos.."] => "..tostring(val))
	                    end
	                end
	            else
	                print(indent..tostring(t))
	            end
	        end
	    end
	    if (type(t)=="table") then
	        print(tostring(t).." {")
	        sub_print_r(t,"  ")
	        print("}")
	    else
	        sub_print_r(t,"  ")
	    end
	    print()
	else
		print(t)
	end
end


--/*---------------------------------------------------------------------------*/--

return strings

