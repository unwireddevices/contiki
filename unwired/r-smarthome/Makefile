.PHONY: all clean smokesensor root root-CLEAN smokesensor-CLEAN firebutton firebutton-CLEAN

all: root-CLEAN smokesensor-CLEAN firebutton-CLEAN

ifndef CPU
  CPU = cc13xx
  ${info Not CPU define, use: "CPU=cc26xx" or "CPU=cc13xx". Set default: $(CPU)}
endif

include ../Makefile_global_unwired

$(eval $(call generate-targets,smokesensor,0x04A00001,0x0B))
$(eval $(call generate-targets,firebutton,0x04A10001,0x0B))
$(eval $(call generate-targets,root,0x04FF0001,0x0C))


# 0x01FF0001 — r-smarthome(04), root(FF), version 0001
# 0x01A00001 — r-smarthome(04), smokesensor(A0), version 0001
# 0x01A00001 — r-smarthome(04), firebutton(A1), version 0001
