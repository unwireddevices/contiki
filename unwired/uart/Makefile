.PHONY: all clean dimmer relay root uart root-CLEAN uart-CLEAN relay-CLEAN dimmer-CLEAN

all: root-CLEAN relay-CLEAN dimmer-CLEAN uart-CLEAN

ifndef CPU
  CPU = cc26xx
  ${info Not CPU define, use: "CPU=cc26xx" or "CPU=cc13xx". Set default: $(CPU)}
endif

include ../Makefile_global_unwired

$(eval $(call generate-targets,root,0x00FF0001,0x0C))
$(eval $(call generate-targets,uart,0x00A10001,0x0C))
$(eval $(call generate-targets,relay,0x00A20001,0x0C))

# 0x00FF0001 — unwired smarthome(00), root(FF), version 0001
# 0x00A10001 — unwired smarthome(00), uart(A1), version 0001
# 0x00A20001 — unwired smarthome(00), relay(A2), version 0001