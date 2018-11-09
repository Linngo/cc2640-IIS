# invoke SourceDir generated makefile for app_ble.prm3
app_ble.prm3: .libraries,app_ble.prm3
.libraries,app_ble.prm3: package/cfg/app_ble_prm3.xdl
	$(MAKE) -f C:\ti\simplelink_cc2640r2_sdk_ble_example_pack_1_50_00_62\examples\rtos\CC2640R2_LAUNCHXL\bleapps\hid_emu_kbd\tirtos/src/makefile.libs

clean::
	$(MAKE) -f C:\ti\simplelink_cc2640r2_sdk_ble_example_pack_1_50_00_62\examples\rtos\CC2640R2_LAUNCHXL\bleapps\hid_emu_kbd\tirtos/src/makefile.libs clean

