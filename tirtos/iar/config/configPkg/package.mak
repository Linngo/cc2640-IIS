#
#  Do not edit this file.  This file is generated from 
#  package.bld.  Any modifications to this file will be 
#  overwritten whenever makefiles are re-generated.
#

unexport MAKEFILE_LIST
MK_NOGENDEPS := $(filter clean,$(MAKECMDGOALS))
override PKGDIR = configPkg
XDCINCS = -I. -I$(strip $(subst ;, -I,$(subst $(space),\$(space),$(XPKGPATH))))
XDCCFGDIR = package/cfg/

#
# The following dependencies ensure package.mak is rebuilt
# in the event that some included BOM script changes.
#
ifneq (clean,$(MAKECMDGOALS))
C:/ti/xdctools_3_50_03_33_core/packages/xdc/utils.js:
package.mak: C:/ti/xdctools_3_50_03_33_core/packages/xdc/utils.js
C:/ti/xdctools_3_50_03_33_core/packages/xdc/xdc.tci:
package.mak: C:/ti/xdctools_3_50_03_33_core/packages/xdc/xdc.tci
C:/ti/xdctools_3_50_03_33_core/packages/xdc/template.xs:
package.mak: C:/ti/xdctools_3_50_03_33_core/packages/xdc/template.xs
C:/ti/xdctools_3_50_03_33_core/packages/xdc/om2.xs:
package.mak: C:/ti/xdctools_3_50_03_33_core/packages/xdc/om2.xs
C:/ti/xdctools_3_50_03_33_core/packages/xdc/xmlgen.xs:
package.mak: C:/ti/xdctools_3_50_03_33_core/packages/xdc/xmlgen.xs
C:/ti/xdctools_3_50_03_33_core/packages/xdc/xmlgen2.xs:
package.mak: C:/ti/xdctools_3_50_03_33_core/packages/xdc/xmlgen2.xs
C:/ti/xdctools_3_50_03_33_core/packages/xdc/Warnings.xs:
package.mak: C:/ti/xdctools_3_50_03_33_core/packages/xdc/Warnings.xs
C:/ti/xdctools_3_50_03_33_core/packages/xdc/IPackage.xs:
package.mak: C:/ti/xdctools_3_50_03_33_core/packages/xdc/IPackage.xs
C:/ti/xdctools_3_50_03_33_core/packages/xdc/package.xs:
package.mak: C:/ti/xdctools_3_50_03_33_core/packages/xdc/package.xs
C:/ti/xdctools_3_50_03_33_core/packages/xdc/services/global/Clock.xs:
package.mak: C:/ti/xdctools_3_50_03_33_core/packages/xdc/services/global/Clock.xs
C:/ti/xdctools_3_50_03_33_core/packages/xdc/services/global/Trace.xs:
package.mak: C:/ti/xdctools_3_50_03_33_core/packages/xdc/services/global/Trace.xs
C:/ti/xdctools_3_50_03_33_core/packages/xdc/bld/bld.js:
package.mak: C:/ti/xdctools_3_50_03_33_core/packages/xdc/bld/bld.js
C:/ti/xdctools_3_50_03_33_core/packages/xdc/bld/BuildEnvironment.xs:
package.mak: C:/ti/xdctools_3_50_03_33_core/packages/xdc/bld/BuildEnvironment.xs
C:/ti/xdctools_3_50_03_33_core/packages/xdc/bld/PackageContents.xs:
package.mak: C:/ti/xdctools_3_50_03_33_core/packages/xdc/bld/PackageContents.xs
C:/ti/xdctools_3_50_03_33_core/packages/xdc/bld/_gen.xs:
package.mak: C:/ti/xdctools_3_50_03_33_core/packages/xdc/bld/_gen.xs
C:/ti/xdctools_3_50_03_33_core/packages/xdc/bld/Library.xs:
package.mak: C:/ti/xdctools_3_50_03_33_core/packages/xdc/bld/Library.xs
C:/ti/xdctools_3_50_03_33_core/packages/xdc/bld/Executable.xs:
package.mak: C:/ti/xdctools_3_50_03_33_core/packages/xdc/bld/Executable.xs
C:/ti/xdctools_3_50_03_33_core/packages/xdc/bld/Repository.xs:
package.mak: C:/ti/xdctools_3_50_03_33_core/packages/xdc/bld/Repository.xs
C:/ti/xdctools_3_50_03_33_core/packages/xdc/bld/Configuration.xs:
package.mak: C:/ti/xdctools_3_50_03_33_core/packages/xdc/bld/Configuration.xs
C:/ti/xdctools_3_50_03_33_core/packages/xdc/bld/Script.xs:
package.mak: C:/ti/xdctools_3_50_03_33_core/packages/xdc/bld/Script.xs
C:/ti/xdctools_3_50_03_33_core/packages/xdc/bld/Manifest.xs:
package.mak: C:/ti/xdctools_3_50_03_33_core/packages/xdc/bld/Manifest.xs
C:/ti/xdctools_3_50_03_33_core/packages/xdc/bld/Utils.xs:
package.mak: C:/ti/xdctools_3_50_03_33_core/packages/xdc/bld/Utils.xs
C:/ti/xdctools_3_50_03_33_core/packages/xdc/bld/ITarget.xs:
package.mak: C:/ti/xdctools_3_50_03_33_core/packages/xdc/bld/ITarget.xs
C:/ti/xdctools_3_50_03_33_core/packages/xdc/bld/ITarget2.xs:
package.mak: C:/ti/xdctools_3_50_03_33_core/packages/xdc/bld/ITarget2.xs
C:/ti/xdctools_3_50_03_33_core/packages/xdc/bld/ITarget3.xs:
package.mak: C:/ti/xdctools_3_50_03_33_core/packages/xdc/bld/ITarget3.xs
C:/ti/xdctools_3_50_03_33_core/packages/xdc/bld/ITargetFilter.xs:
package.mak: C:/ti/xdctools_3_50_03_33_core/packages/xdc/bld/ITargetFilter.xs
C:/ti/xdctools_3_50_03_33_core/packages/xdc/bld/package.xs:
package.mak: C:/ti/xdctools_3_50_03_33_core/packages/xdc/bld/package.xs
package.mak: config.bld
C:/ti/simplelink_cc2640r2_sdk_1_50_00_58/kernel/tirtos/packages/iar/targets/arm/ITarget.xs:
package.mak: C:/ti/simplelink_cc2640r2_sdk_1_50_00_58/kernel/tirtos/packages/iar/targets/arm/ITarget.xs
C:/ti/simplelink_cc2640r2_sdk_1_50_00_58/kernel/tirtos/packages/iar/targets/arm/package.xs:
package.mak: C:/ti/simplelink_cc2640r2_sdk_1_50_00_58/kernel/tirtos/packages/iar/targets/arm/package.xs
package.mak: package.bld
C:/ti/xdctools_3_50_03_33_core/packages/xdc/tools/configuro/template/compiler.opt.xdt:
package.mak: C:/ti/xdctools_3_50_03_33_core/packages/xdc/tools/configuro/template/compiler.opt.xdt
C:/ti/xdctools_3_50_03_33_core/packages/xdc/services/io/File.xs:
package.mak: C:/ti/xdctools_3_50_03_33_core/packages/xdc/services/io/File.xs
C:/ti/xdctools_3_50_03_33_core/packages/xdc/services/io/package.xs:
package.mak: C:/ti/xdctools_3_50_03_33_core/packages/xdc/services/io/package.xs
C:/ti/xdctools_3_50_03_33_core/packages/xdc/tools/configuro/template/compiler.defs.xdt:
package.mak: C:/ti/xdctools_3_50_03_33_core/packages/xdc/tools/configuro/template/compiler.defs.xdt
C:/ti/xdctools_3_50_03_33_core/packages/xdc/tools/configuro/template/custom.mak.exe.xdt:
package.mak: C:/ti/xdctools_3_50_03_33_core/packages/xdc/tools/configuro/template/custom.mak.exe.xdt
C:/ti/xdctools_3_50_03_33_core/packages/xdc/tools/configuro/template/package.xs.xdt:
package.mak: C:/ti/xdctools_3_50_03_33_core/packages/xdc/tools/configuro/template/package.xs.xdt
endif

iar.targets.arm.M3.rootDir ?= C:/IAR_Systems/Embedded_Workbench_8.0/arm
iar.targets.arm.packageBase ?= C:/ti/simplelink_cc2640r2_sdk_1_50_00_58/kernel/tirtos/packages/iar/targets/arm/
.PRECIOUS: $(XDCCFGDIR)/%.orm3
.PHONY: all,rm3 .dlls,rm3 .executables,rm3 test,rm3
all,rm3: .executables,rm3
.executables,rm3: .libraries,rm3
.executables,rm3: .dlls,rm3
.dlls,rm3: .libraries,rm3
.libraries,rm3: .interfaces
	@$(RM) $@
	@$(TOUCH) "$@"

.help::
	@$(ECHO) xdc test,rm3
	@$(ECHO) xdc .executables,rm3
	@$(ECHO) xdc .libraries,rm3
	@$(ECHO) xdc .dlls,rm3


all: .executables 
.executables: .libraries .dlls
.libraries: .interfaces

PKGCFGS := $(wildcard package.xs) package/build.cfg
.interfaces: package/package.xdc.inc package/package.defs.h package.xdc $(PKGCFGS)

-include package/package.xdc.dep
package/%.xdc.inc package/%_configPkg.c package/%.defs.h: %.xdc $(PKGCFGS)
	@$(MSG) generating interfaces for package configPkg" (because $@ is older than $(firstword $?))" ...
	$(XSRUN) -f xdc/services/intern/cmd/build.xs $(MK_IDLOPTS) -m package/package.xdc.dep -i package/package.xdc.inc package.xdc

.dlls,rm3 .dlls: app_ble.prm3

-include package/cfg/app_ble_prm3.mak
-include package/cfg/app_ble_prm3.cfg.mak
ifeq (,$(MK_NOGENDEPS))
-include package/cfg/app_ble_prm3.dep
endif
app_ble.prm3: package/cfg/app_ble_prm3.xdl
	@


ifeq (,$(wildcard .libraries,rm3))
app_ble.prm3 package/cfg/app_ble_prm3.c: .libraries,rm3
endif

package/cfg/app_ble_prm3.c package/cfg/app_ble_prm3.h package/cfg/app_ble_prm3.xdl: override _PROG_NAME := app_ble.xrm3
package/cfg/app_ble_prm3.c: package/cfg/app_ble_prm3.cfg
package/cfg/app_ble_prm3.xdc.inc: package/cfg/app_ble_prm3.xdl
package/cfg/app_ble_prm3.xdl package/cfg/app_ble_prm3.c: .interfaces

clean:: clean,rm3
	-$(RM) package/cfg/app_ble_prm3.cfg
	-$(RM) package/cfg/app_ble_prm3.dep
	-$(RM) package/cfg/app_ble_prm3.c
	-$(RM) package/cfg/app_ble_prm3.xdc.inc

clean,rm3::
	-$(RM) app_ble.prm3
.executables,rm3 .executables: app_ble.xrm3

app_ble.xrm3: |app_ble.prm3

-include package/cfg/app_ble.xrm3.mak
app_ble.xrm3: package/cfg/app_ble_prm3.orm3 
	$(RM) $@
	@$(MSG) lnkrm3 $@ ...
	$(RM) $(XDCCFGDIR)/$@.map
	LC_ALL=C $(iar.targets.arm.M3.rootDir)/bin/ilinkarm -o C:/ti/simplelink_cc2640r2_sdk_ble_example_pack_1_50_00_62/examples/rtos/CC2640R2_LAUNCHXL/bleapps/hid_emu_kbd/tirtos/iar/app/FlashROM_StackLibrary/Exe/ble_hid_emu_kbd_cc2640r2lp_app_FlashROM_StackLibrary.out --config_def CC2650=2 --config_def FLASH_ROM_BUILD=2 --map C:/ti/simplelink_cc2640r2_sdk_ble_example_pack_1_50_00_62/examples/rtos/CC2640R2_LAUNCHXL/bleapps/hid_emu_kbd/tirtos/iar/app/FlashROM_StackLibrary/List/ble_hid_emu_kbd_cc2640r2lp_app_FlashROM_StackLibrary.map --config c:/ti/simplelink_cc2640r2_sdk_1_50_00_58/source/ti/blestack/common/cc26xx/iar/cc26xx_app_and_stack.icf --keep __vector_table -f C:/ti/simplelink_cc2640r2_sdk_ble_example_pack_1_50_00_62/examples/rtos/CC2640R2_LAUNCHXL/bleapps/hid_emu_kbd/tirtos/iar/app/../config/configPkg/linker.cmd -f C:/ti/simplelink_cc2640r2_sdk_ble_example_pack_1_50_00_62/examples/rtos/CC2640R2_LAUNCHXL/bleapps/hid_emu_kbd/tirtos/iar/app/../config/lib_linker.cmd C:/ti/simplelink_cc2640r2_sdk_ble_example_pack_1_50_00_62/examples/rtos/CC2640R2_LAUNCHXL/bleapps/hid_emu_kbd/tirtos/iar/app/../config/ble_r2.symbols c:/ti/simplelink_cc2640r2_sdk_1_50_00_58/source/ti/devices/cc26x0r2/driverlib/bin/iar/driverlib.lib c:/ti/simplelink_cc2640r2_sdk_1_50_00_58/kernel/tirtos/packages/ti/dpl/lib/dpl_cc26x0r2.arm3 c:/ti/simplelink_cc2640r2_sdk_1_50_00_58/source/ti/drivers/lib/drivers_cc26x0r2.arm3 c:/ti/simplelink_cc2640r2_sdk_1_50_00_58/source/ti/drivers/rf/lib/rf_singleMode_cc26x0r2.arm3 c:/ti/simplelink_cc2640r2_sdk_1_50_00_58/source/ti/display/lib/display.arm3 c:/ti/simplelink_cc2640r2_sdk_1_50_00_58/source/ti/grlib/lib/grlib.arm3 c:/ti/simplelink_cc2640r2_sdk_1_50_00_58/source/ti/mw/lcd/lib/lcd.arm3 --entry __iar_program_start --vfe --text_out locale --silent -o $@ package/cfg/app_ble_prm3.orm3 -f package/cfg/app_ble_prm3.xdl  --semihosting=iar_breakpoint  --cpu=Cortex-M3 --map $(XDCCFGDIR)/$@.map  --redirect _Printf=_PrintfSmall --redirect _Scanf=_ScanfSmall 
	

app_ble.test test,rm3 test: app_ble.xrm3.test

app_ble.xrm3.test:: app_ble.xrm3
ifeq (,$(_TESTLEVEL))
	@$(MAKE) -R -r --no-print-directory -f $(XDCROOT)/packages/xdc/bld/xdc.mak _TESTLEVEL=1 app_ble.xrm3.test
else
	@$(MSG) running $<  ...
	$(call EXEC.app_ble.xrm3, ) 
endif

clean,rm3::
	-$(RM) $(wildcard .tmp,app_ble.xrm3,*)


clean:: clean,rm3

clean,rm3::
	-$(RM) app_ble.xrm3
%,copy:
	@$(if $<,,$(MSG) don\'t know how to build $*; exit 1)
	@$(MSG) cp $< $@
	$(RM) $@
	$(CP) $< $@
app_ble_prm3.orm3,copy : package/cfg/app_ble_prm3.orm3
app_ble_prm3.srm3,copy : package/cfg/app_ble_prm3.srm3

$(XDCCFGDIR)%.c $(XDCCFGDIR)%.h $(XDCCFGDIR)%.xdl: $(XDCCFGDIR)%.cfg $(XDCROOT)/packages/xdc/cfg/Main.xs | .interfaces
	@$(MSG) "configuring $(_PROG_NAME) from $< ..."
	$(CONFIG) $(_PROG_XSOPTS) xdc.cfg $(_PROG_NAME) $(XDCCFGDIR)$*.cfg $(XDCCFGDIR)$*

.PHONY: release,configPkg
ifeq (,$(MK_NOGENDEPS))
-include package/rel/configPkg.tar.dep
endif
package/rel/configPkg/configPkg/package/package.rel.xml: package/package.bld.xml
package/rel/configPkg/configPkg/package/package.rel.xml: package/build.cfg
package/rel/configPkg/configPkg/package/package.rel.xml: package/package.xdc.inc
package/rel/configPkg/configPkg/package/package.rel.xml: .force
	@$(MSG) generating external release references $@ ...
	$(XS) $(JSENV) -f $(XDCROOT)/packages/xdc/bld/rel.js $(MK_RELOPTS) . $@

configPkg.tar: package/rel/configPkg.xdc.inc package/rel/configPkg/configPkg/package/package.rel.xml
	@$(MSG) making release file $@ "(because of $(firstword $?))" ...
	-$(RM) $@
	$(call MKRELTAR,package/rel/configPkg.xdc.inc,package/rel/configPkg.tar.dep)


release release,configPkg: all configPkg.tar
clean:: .clean
	-$(RM) configPkg.tar
	-$(RM) package/rel/configPkg.xdc.inc
	-$(RM) package/rel/configPkg.tar.dep

clean:: .clean
	-$(RM) .libraries $(wildcard .libraries,*)
clean:: 
	-$(RM) .dlls $(wildcard .dlls,*)
#
# The following clean rule removes user specified
# generated files or directories.
#

ifneq (clean,$(MAKECMDGOALS))
ifeq (,$(wildcard package))
    $(shell $(MKDIR) package)
endif
ifeq (,$(wildcard package/cfg))
    $(shell $(MKDIR) package/cfg)
endif
ifeq (,$(wildcard package/lib))
    $(shell $(MKDIR) package/lib)
endif
ifeq (,$(wildcard package/rel))
    $(shell $(MKDIR) package/rel)
endif
ifeq (,$(wildcard package/internal))
    $(shell $(MKDIR) package/internal)
endif
endif
clean::
	-$(RMDIR) package

include custom.mak
