## THIS IS A GENERATED FILE -- DO NOT EDIT
.configuro: .libraries,em4f linker.cmd package/cfg/empty_min_pem4f.oem4f

# To simplify configuro usage in makefiles:
#     o create a generic linker command file name 
#     o set modification times of compiler.opt* files to be greater than
#       or equal to the generated config header
#
linker.cmd: package/cfg/empty_min_pem4f.xdl
	$(SED) 's"^\"\(package/cfg/empty_min_pem4fcfg.cmd\)\"$""\"C:/Users/arnau/workspace_v10/PID_MSP_EXP432P401R_tirtos_ccs2/.config/xconfig_empty_min/\1\""' package/cfg/empty_min_pem4f.xdl > $@
	-$(SETDATE) -r:max package/cfg/empty_min_pem4f.h compiler.opt compiler.opt.defs
