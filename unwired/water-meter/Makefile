.PHONY: all clean wmeter root root-CLEAN wmeter-CLEAN

all: root-CLEAN wmeter-CLEAN

ifndef CPU
  CPU = cc13xx
  ${info Not CPU define, use: "CPU=cc26xx" or "CPU=cc13xx". Set default: $(CPU)}
endif

include ../Makefile_global_unwired

$(eval $(call generate-targets,wmeter,0x03A00001,0x0B))
$(eval $(call generate-targets,root,0x03FF0001,0x0C))

# 0x01FF0001 — water-meter(water-meter, 03), root(FF), version 0001
# 0x01A00001 — water-meter(water-meter, 03), wmeter(A0), version 0001
