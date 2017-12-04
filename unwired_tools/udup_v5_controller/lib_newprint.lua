local newprint = {}

function newprint.color(color)
	if (color == "red") then
		io.write(string.char(27,91,51,49,109))
	elseif (color == "none") then
		io.write(string.char(27,91,48,109))
	end
end

--/*---------------------------------------------------------------------------*/--

function newprint.print(data, ...)
	io.write(data, ...)
	io.write("\n")
	io.flush()
end

--/*---------------------------------------------------------------------------*/--

function newprint.print_n(data, ...)
	io.write(data, ...)
	io.flush()
end

function newprint.print_red(data, ...)
	newprint.color("red")
	print(data, ...)
	newprint.color("none")
end

--/*---------------------------------------------------------------------------*/--

return newprint
