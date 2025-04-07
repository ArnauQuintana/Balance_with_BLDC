# invoke SourceDir generated makefile for empty_min.pem4f
empty_min.pem4f: .libraries,empty_min.pem4f
.libraries,empty_min.pem4f: package/cfg/empty_min_pem4f.xdl
	$(MAKE) -f C:\Users\arnau\workspace_v10\PID_MSP_EXP432P401R_tirtos_ccs2/src/makefile.libs

clean::
	$(MAKE) -f C:\Users\arnau\workspace_v10\PID_MSP_EXP432P401R_tirtos_ccs2/src/makefile.libs clean

