## THIS IS A GENERATED FILE -- DO NOT EDIT
.configuro: .libraries,em4f linker.cmd package/cfg/uartlogging_pem4f.oem4f

# To simplify configuro usage in makefiles:
#     o create a generic linker command file name 
#     o set modification times of compiler.opt* files to be greater than
#       or equal to the generated config header
#
linker.cmd: package/cfg/uartlogging_pem4f.xdl
	$(SED) 's"^\"\(package/cfg/uartlogging_pem4fcfg.cmd\)\"$""\"D:/Usuarios/usuari/Documents/LSE2/curs2020-21/projectes_CCS1011/uartlogging_MSP_EXP432P401R_tirtos_ccs/.config/xconfig_uartlogging/\1\""' package/cfg/uartlogging_pem4f.xdl > $@
	-$(SETDATE) -r:max package/cfg/uartlogging_pem4f.h compiler.opt compiler.opt.defs
