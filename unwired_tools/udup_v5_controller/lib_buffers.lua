local buffers = {}

--/*---------------------------------------------------------------------------*/--

function buffers.add_byte_to_buffer(buffer, byte)
   table.insert(buffer, byte)
   while (#buffer > 100) do
      table.remove(buffer, 1)
   end
end

--/*---------------------------------------------------------------------------*/--

function buffers.clean_buffer(buffer)
   for i = 1, #buffer do
      table.insert(buffer, 0)
   end
end

--/*---------------------------------------------------------------------------*/--

function buffers.get_buffer(buffer)
   return table.concat(buffer)
end

--/*---------------------------------------------------------------------------*/--

function buffers.create_buffer()
   local buffer = {}
   return buffer
end

--/*---------------------------------------------------------------------------*/--

return buffers
