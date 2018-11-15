.PHONY: all clean root pwm_fet button lit gpio root-CLEAN pwm_fet-CLEAN button-CLEAN lit-CLEAN gpio-CLEAN

all: root-CLEAN pwm_fet-CLEAN button-CLEAN lit-CLEAN gpio-CLEAN

ifndef CPU
  CPU = cc26xx
  ${info Not CPU define, use: "CPU=cc26xx" or "CPU=cc13xx". Set default: $(CPU)}
endif

include ../Makefile_global_unwired

$(eval $(call generate-targets,root,0x00FF0001,0x0C))
$(eval $(call generate-targets,pwm_fet,0x00A10001,0x0C))
$(eval $(call generate-targets,button,0x00A20001,0x0C))
$(eval $(call generate-targets,lit,0x00A30001,0x0C))
$(eval $(call generate-targets,gpio,0x00A40001,0x0C))

# 0x00FF0001 — unwired smartoffice(00), root(FF), version 0001
# 0x00A10001 — unwired smartoffice(00), pwm_fet(A1), version 0001
# 0x00A20001 — unwired smartoffice(00), button(A2), version 0001
# 0x00A30001 — unwired smartoffice(00), lit(A3), version 0001
# 0x00A40001 — unwired smartoffice(00), gpio(A4), version 0001