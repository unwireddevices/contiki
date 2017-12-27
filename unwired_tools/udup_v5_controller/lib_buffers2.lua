local buffers = {}
local buffer_size = 0
local buffer = ""

--/*---------------------------------------------------------------------------*/--

function buffers.add_byte_to_buffer(byte)
   buffer = buffer..byte
   buffer = string.sub(buffer, -buffer_size)
end

--/*---------------------------------------------------------------------------*/--

function buffers.clean_buffer()
   buffer = ""
end

--/*---------------------------------------------------------------------------*/--

function buffers.get_buffer()
   return buffer
end

--/*---------------------------------------------------------------------------*/--

function buffers.create_buffer(size)
   buffer_size = size
   buffer = ""
end

--/*---------------------------------------------------------------------------*/--

return buffers
