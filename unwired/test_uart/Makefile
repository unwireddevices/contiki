.PHONY: all clean root test_uart root-CLEAN test_uart-CLEAN 

all: root-CLEAN test_uart-CLEAN

ifndef CPU
  CPU = cc26xx
  ${info Not CPU define, use: "CPU=cc26xx" or "CPU=cc13xx". Set default: $(CPU)}
endif

include ../Makefile_global_unwired

$(eval $(call generate-targets,root,0x00FF0001,0x0C))
$(eval $(call generate-targets,test_uart,0x00A10001,0x0C))

# 0x00FF0001 — unwired smartoffice(00), root(FF), version 0001
# 0x00A10001 — unwired smartoffice(00), test_uart(A1), version 0001