################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
build-1383382513: ../simple_peripheral.syscfg
	@echo 'Building file: "$<"'
	@echo 'Invoking: SysConfig'
	"/Applications/ti/sysconfig_1.15.0/sysconfig_cli.sh" -s "/Applications/ti/simplelink_cc13xx_cc26xx_sdk_6_40_00_13/.metadata/product.json" --script "/Users/michaelliesenberg/workspace_CC2652P/simple_peripheral_CC1352P_2_LAUNCHXL_tirtos7_ticlang/simple_peripheral.syscfg" -o "syscfg" --compiler ticlang
	@echo 'Finished building: "$<"'
	@echo ' '

syscfg/ti_ble_config.h: build-1383382513 ../simple_peripheral.syscfg
syscfg/ti_ble_config.c: build-1383382513
syscfg/ti_build_config.opt: build-1383382513
syscfg/ti_ble_app_config.opt: build-1383382513
syscfg/ti_devices_config.c: build-1383382513
syscfg/ti_radio_config.c: build-1383382513
syscfg/ti_radio_config.h: build-1383382513
syscfg/ti_drivers_config.c: build-1383382513
syscfg/ti_drivers_config.h: build-1383382513
syscfg/ti_utils_build_linker.cmd.genlibs: build-1383382513
syscfg/syscfg_c.rov.xs: build-1383382513
syscfg/ti_utils_runtime_model.gv: build-1383382513
syscfg/ti_utils_runtime_Makefile: build-1383382513
syscfg/ti_sysbios_config.h: build-1383382513
syscfg/ti_sysbios_config.c: build-1383382513
syscfg/: build-1383382513

syscfg/%.o: ./syscfg/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"/Applications/ti/ccs1120/ccs/tools/compiler/ti-cgt-armllvm_1.3.1.LTS/bin/tiarmclang" -c @"/Users/michaelliesenberg/workspace_CC2652P/simple_peripheral_CC1352P_2_LAUNCHXL_tirtos7_ticlang/Release/syscfg/ti_ble_app_config.opt" @"/Users/michaelliesenberg/workspace_CC2652P/simple_peripheral_CC1352P_2_LAUNCHXL_tirtos7_ticlang/Release/syscfg/ti_build_config.opt" @"/Applications/ti/simplelink_cc13xx_cc26xx_sdk_6_40_00_13/source/ti/ble5stack/config/build_components.opt" @"/Applications/ti/simplelink_cc13xx_cc26xx_sdk_6_40_00_13/source/ti/ble5stack/config/factory_config.opt"  -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -mlittle-endian -mthumb -O0 -I"/Users/michaelliesenberg/workspace_CC2652P/simple_peripheral_CC1352P_2_LAUNCHXL_tirtos7_ticlang" -I"/Users/michaelliesenberg/workspace_CC2652P/simple_peripheral_CC1352P_2_LAUNCHXL_tirtos7_ticlang/Release" -I"/Users/michaelliesenberg/workspace_CC2652P/simple_peripheral_CC1352P_2_LAUNCHXL_tirtos7_ticlang/Application" -I"/Applications/ti/simplelink_cc13xx_cc26xx_sdk_6_40_00_13/source" -I"/Applications/ti/simplelink_cc13xx_cc26xx_sdk_6_40_00_13/source/ti" -I"/Applications/ti/simplelink_cc13xx_cc26xx_sdk_6_40_00_13/source/ti/ble5stack" -I"/Applications/ti/simplelink_cc13xx_cc26xx_sdk_6_40_00_13/source/ti/ble5stack/controller/cc26xx/inc" -I"/Applications/ti/simplelink_cc13xx_cc26xx_sdk_6_40_00_13/source/ti/ble5stack/inc" -I"/Applications/ti/simplelink_cc13xx_cc26xx_sdk_6_40_00_13/source/ti/ble5stack/rom" -I"/Applications/ti/simplelink_cc13xx_cc26xx_sdk_6_40_00_13/source/ti/ble5stack/common/cc26xx" -I"/Applications/ti/simplelink_cc13xx_cc26xx_sdk_6_40_00_13/source/ti/ble5stack/icall/inc" -I"/Applications/ti/simplelink_cc13xx_cc26xx_sdk_6_40_00_13/source/ti/ble5stack/hal/src/target/_common" -I"/Applications/ti/simplelink_cc13xx_cc26xx_sdk_6_40_00_13/source/ti/ble5stack/common/cc26xx/npi/stack" -I"/Applications/ti/simplelink_cc13xx_cc26xx_sdk_6_40_00_13/source/ti/ble5stack/hal/src/inc" -I"/Applications/ti/simplelink_cc13xx_cc26xx_sdk_6_40_00_13/source/ti/ble5stack/heapmgr" -I"/Applications/ti/simplelink_cc13xx_cc26xx_sdk_6_40_00_13/source/ti/ble5stack/profiles/dev_info" -I"/Applications/ti/simplelink_cc13xx_cc26xx_sdk_6_40_00_13/source/ti/ble5stack/profiles/simple_profile" -I"/Applications/ti/simplelink_cc13xx_cc26xx_sdk_6_40_00_13/source/ti/ble5stack/icall/src/inc" -I"/Applications/ti/simplelink_cc13xx_cc26xx_sdk_6_40_00_13/source/ti/ble5stack/osal/src/inc" -I"/Applications/ti/simplelink_cc13xx_cc26xx_sdk_6_40_00_13/source/ti/ble5stack/services/src/saddr" -I"/Applications/ti/simplelink_cc13xx_cc26xx_sdk_6_40_00_13/source/ti/ble5stack/services/src/sdata" -I"/Applications/ti/simplelink_cc13xx_cc26xx_sdk_6_40_00_13/source/ti/common/nv" -I"/Applications/ti/simplelink_cc13xx_cc26xx_sdk_6_40_00_13/source/ti/common/cc26xx" -I"/Applications/ti/simplelink_cc13xx_cc26xx_sdk_6_40_00_13/source/ti/ble5stack/icall/src" -I"/Applications/ti/simplelink_cc13xx_cc26xx_sdk_6_40_00_13/source/ti/ble5stack/common/cc26xx/rcosc" -I"/Applications/ti/simplelink_cc13xx_cc26xx_sdk_6_40_00_13/source/ti/ble5stack/npi/src" -I"/Applications/ti/simplelink_cc13xx_cc26xx_sdk_6_40_00_13/source/ti/ble5stack/npi/src/inc" -I"/Applications/ti/simplelink_cc13xx_cc26xx_sdk_6_40_00_13/source/ti/devices/cc13x2_cc26x2" -I"/Applications/ti/simplelink_cc13xx_cc26xx_sdk_6_40_00_13/kernel/tirtos7/packages" -I"/Applications/ti/simplelink_cc13xx_cc26xx_sdk_6_40_00_13/source/ti/posix/ticlang" -DPLUS_BROADCASTER_VALUES_NEW -DBNO055 -DNPI_FLOW_CTRL=0 -Dbattery_capacity=1200 -DDeviceFamily_CC13X2 -DFLASH_ROM_BUILD -DNVOCMP_NWSAMEITEM=1 -DHEAPMGR_CONFIG=0x80 -DHEAPMGR_SIZE=0x0 -gdwarf-3 -march=armv7e-m -MMD -MP -MF"syscfg/$(basename $(<F)).d_raw" -MT"$(@)" -I"/Users/michaelliesenberg/workspace_CC2652P/simple_peripheral_CC1352P_2_LAUNCHXL_tirtos7_ticlang/Release/syscfg" -std=gnu9x $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


