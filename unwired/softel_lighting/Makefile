.PHONY: all clean softel_lighting softel_lighting-CLEAN

all: softel_lighting-CLEAN

ifndef CPU
  CPU = cc13xx
  ${info Not CPU define, use: "CPU=cc26xx" or "CPU=cc13xx". Set default: $(CPU)}
endif

include ../Makefile_global_unwired

$(eval $(call generate-targets,softel_lighting,0x00FF0001,0x0C))

# 0x00FF0001 — unwired smartoffice(00), root(FF), version 0001