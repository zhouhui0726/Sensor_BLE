## THIS IS A GENERATED FILE -- DO NOT EDIT
.configuro: .libraries,rm3 linker.cmd package/cfg/appBLE_prm3.orm3

# To simplify configuro usage in makefiles:
#     o create a generic linker command file name 
#     o set modification times of compiler.opt* files to be greater than
#       or equal to the generated config header
#
linker.cmd: package/cfg/appBLE_prm3.xdl
	$(SED) 's"^\"\(package/cfg/appBLE_prm3cfg.cmd\)\"$""\"C:/ti/simplelink/ble_cc26xx_2_01_00_44423/Projects/ble/JSensor/JSensor-T/CC26xx/IAR/Application/CC2640/configPkg/\1\""' package/cfg/appBLE_prm3.xdl > $@
	-$(SETDATE) -r:max package/cfg/appBLE_prm3.h compiler.opt compiler.opt.defs