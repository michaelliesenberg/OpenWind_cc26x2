################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
Application/bim_onchip_main.o: /Applications/ti/simplelink_cc13xx_cc26xx_sdk_6_40_00_13/source/ti/common/bim/bim_onchip_main.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"/Users/michaelliesenberg/ti/ti_cgt_tiarmclang_1.2.1.STS/bin/tiarmclang" -c -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -mlittle-endian -mthumb -Oz -I"/Users/michaelliesenberg/workspace_CC2652P/bim_onchip_CC1352P_2_LAUNCHXL_nortos_ticlang" -I"/Users/michaelliesenberg/ti/ti_cgt_tiarmclang_1.2.1.STS/include" -I"/Applications/ti/simplelink_cc13xx_cc26xx_sdk_6_40_00_13/source" -I"/Applications/ti/simplelink_cc13xx_cc26xx_sdk_6_40_00_13/source/ti/common/flash/no_rtos/extFlash" -I"/Applications/ti/simplelink_cc13xx_cc26xx_sdk_6_40_00_13/source/ti/devices/cc13x2_cc26x2/startup_files" -DDeviceFamily_CC13X2 -DSET_CCFG_IMAGE_VALID_CONF_IMAGE_VALID=0x56000 -DSET_CCFG_MODE_CONF_XOSC_CAP_MOD=0x0 -DSET_CCFG_MODE_CONF_XOSC_CAPARRAY_DELTA=-63 -DBIM_ONCHIP -DxSECURITY -DDEBUG_BIM -gdwarf-3 -fshort-enums -munaligned-access -funsigned-char -fcommon -ffunction-sections -fdata-sections -march=armv7e-m -MMD -MP -MF"Application/$(basename $(<F)).d_raw" -MT"$(@)" -std=gnu9x $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

Application/bim_util.o: /Applications/ti/simplelink_cc13xx_cc26xx_sdk_6_40_00_13/source/ti/common/cc26xx/bim/bim_util.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"/Users/michaelliesenberg/ti/ti_cgt_tiarmclang_1.2.1.STS/bin/tiarmclang" -c -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -mlittle-endian -mthumb -Oz -I"/Users/michaelliesenberg/workspace_CC2652P/bim_onchip_CC1352P_2_LAUNCHXL_nortos_ticlang" -I"/Users/michaelliesenberg/ti/ti_cgt_tiarmclang_1.2.1.STS/include" -I"/Applications/ti/simplelink_cc13xx_cc26xx_sdk_6_40_00_13/source" -I"/Applications/ti/simplelink_cc13xx_cc26xx_sdk_6_40_00_13/source/ti/common/flash/no_rtos/extFlash" -I"/Applications/ti/simplelink_cc13xx_cc26xx_sdk_6_40_00_13/source/ti/devices/cc13x2_cc26x2/startup_files" -DDeviceFamily_CC13X2 -DSET_CCFG_IMAGE_VALID_CONF_IMAGE_VALID=0x56000 -DSET_CCFG_MODE_CONF_XOSC_CAP_MOD=0x0 -DSET_CCFG_MODE_CONF_XOSC_CAPARRAY_DELTA=-63 -DBIM_ONCHIP -DxSECURITY -DDEBUG_BIM -gdwarf-3 -fshort-enums -munaligned-access -funsigned-char -fcommon -ffunction-sections -fdata-sections -march=armv7e-m -MMD -MP -MF"Application/$(basename $(<F)).d_raw" -MT"$(@)" -std=gnu9x $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

Application/ccfg_app.o: /Applications/ti/simplelink_cc13xx_cc26xx_sdk_6_40_00_13/source/ti/common/bim/ccfg_app.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"/Users/michaelliesenberg/ti/ti_cgt_tiarmclang_1.2.1.STS/bin/tiarmclang" -c -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -mlittle-endian -mthumb -Oz -I"/Users/michaelliesenberg/workspace_CC2652P/bim_onchip_CC1352P_2_LAUNCHXL_nortos_ticlang" -I"/Users/michaelliesenberg/ti/ti_cgt_tiarmclang_1.2.1.STS/include" -I"/Applications/ti/simplelink_cc13xx_cc26xx_sdk_6_40_00_13/source" -I"/Applications/ti/simplelink_cc13xx_cc26xx_sdk_6_40_00_13/source/ti/common/flash/no_rtos/extFlash" -I"/Applications/ti/simplelink_cc13xx_cc26xx_sdk_6_40_00_13/source/ti/devices/cc13x2_cc26x2/startup_files" -DDeviceFamily_CC13X2 -DSET_CCFG_IMAGE_VALID_CONF_IMAGE_VALID=0x56000 -DSET_CCFG_MODE_CONF_XOSC_CAP_MOD=0x0 -DSET_CCFG_MODE_CONF_XOSC_CAPARRAY_DELTA=-63 -DBIM_ONCHIP -DxSECURITY -DDEBUG_BIM -gdwarf-3 -fshort-enums -munaligned-access -funsigned-char -fcommon -ffunction-sections -fdata-sections -march=armv7e-m -MMD -MP -MF"Application/$(basename $(<F)).d_raw" -MT"$(@)" -std=gnu9x $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

Application/crc32.o: /Applications/ti/simplelink_cc13xx_cc26xx_sdk_6_40_00_13/source/ti/common/cc26xx/crc/crc32.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"/Users/michaelliesenberg/ti/ti_cgt_tiarmclang_1.2.1.STS/bin/tiarmclang" -c -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -mlittle-endian -mthumb -Oz -I"/Users/michaelliesenberg/workspace_CC2652P/bim_onchip_CC1352P_2_LAUNCHXL_nortos_ticlang" -I"/Users/michaelliesenberg/ti/ti_cgt_tiarmclang_1.2.1.STS/include" -I"/Applications/ti/simplelink_cc13xx_cc26xx_sdk_6_40_00_13/source" -I"/Applications/ti/simplelink_cc13xx_cc26xx_sdk_6_40_00_13/source/ti/common/flash/no_rtos/extFlash" -I"/Applications/ti/simplelink_cc13xx_cc26xx_sdk_6_40_00_13/source/ti/devices/cc13x2_cc26x2/startup_files" -DDeviceFamily_CC13X2 -DSET_CCFG_IMAGE_VALID_CONF_IMAGE_VALID=0x56000 -DSET_CCFG_MODE_CONF_XOSC_CAP_MOD=0x0 -DSET_CCFG_MODE_CONF_XOSC_CAPARRAY_DELTA=-63 -DBIM_ONCHIP -DxSECURITY -DDEBUG_BIM -gdwarf-3 -fshort-enums -munaligned-access -funsigned-char -fcommon -ffunction-sections -fdata-sections -march=armv7e-m -MMD -MP -MF"Application/$(basename $(<F)).d_raw" -MT"$(@)" -std=gnu9x $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

Application/flash_interface_internal.o: /Applications/ti/simplelink_cc13xx_cc26xx_sdk_6_40_00_13/source/ti/common/cc26xx/flash_interface/internal/flash_interface_internal.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"/Users/michaelliesenberg/ti/ti_cgt_tiarmclang_1.2.1.STS/bin/tiarmclang" -c -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -mlittle-endian -mthumb -Oz -I"/Users/michaelliesenberg/workspace_CC2652P/bim_onchip_CC1352P_2_LAUNCHXL_nortos_ticlang" -I"/Users/michaelliesenberg/ti/ti_cgt_tiarmclang_1.2.1.STS/include" -I"/Applications/ti/simplelink_cc13xx_cc26xx_sdk_6_40_00_13/source" -I"/Applications/ti/simplelink_cc13xx_cc26xx_sdk_6_40_00_13/source/ti/common/flash/no_rtos/extFlash" -I"/Applications/ti/simplelink_cc13xx_cc26xx_sdk_6_40_00_13/source/ti/devices/cc13x2_cc26x2/startup_files" -DDeviceFamily_CC13X2 -DSET_CCFG_IMAGE_VALID_CONF_IMAGE_VALID=0x56000 -DSET_CCFG_MODE_CONF_XOSC_CAP_MOD=0x0 -DSET_CCFG_MODE_CONF_XOSC_CAPARRAY_DELTA=-63 -DBIM_ONCHIP -DxSECURITY -DDEBUG_BIM -gdwarf-3 -fshort-enums -munaligned-access -funsigned-char -fcommon -ffunction-sections -fdata-sections -march=armv7e-m -MMD -MP -MF"Application/$(basename $(<F)).d_raw" -MT"$(@)" -std=gnu9x $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

Application/led_debug.o: /Applications/ti/simplelink_cc13xx_cc26xx_sdk_6_40_00_13/source/ti/common/cc26xx/debug/led_debug.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"/Users/michaelliesenberg/ti/ti_cgt_tiarmclang_1.2.1.STS/bin/tiarmclang" -c -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -mlittle-endian -mthumb -Oz -I"/Users/michaelliesenberg/workspace_CC2652P/bim_onchip_CC1352P_2_LAUNCHXL_nortos_ticlang" -I"/Users/michaelliesenberg/ti/ti_cgt_tiarmclang_1.2.1.STS/include" -I"/Applications/ti/simplelink_cc13xx_cc26xx_sdk_6_40_00_13/source" -I"/Applications/ti/simplelink_cc13xx_cc26xx_sdk_6_40_00_13/source/ti/common/flash/no_rtos/extFlash" -I"/Applications/ti/simplelink_cc13xx_cc26xx_sdk_6_40_00_13/source/ti/devices/cc13x2_cc26x2/startup_files" -DDeviceFamily_CC13X2 -DSET_CCFG_IMAGE_VALID_CONF_IMAGE_VALID=0x56000 -DSET_CCFG_MODE_CONF_XOSC_CAP_MOD=0x0 -DSET_CCFG_MODE_CONF_XOSC_CAPARRAY_DELTA=-63 -DBIM_ONCHIP -DxSECURITY -DDEBUG_BIM -gdwarf-3 -fshort-enums -munaligned-access -funsigned-char -fcommon -ffunction-sections -fdata-sections -march=armv7e-m -MMD -MP -MF"Application/$(basename $(<F)).d_raw" -MT"$(@)" -std=gnu9x $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

Application/startup_ticlang.o: /Applications/ti/simplelink_cc13xx_cc26xx_sdk_6_40_00_13/source/ti/devices/cc13x2_cc26x2/startup_files/startup_ticlang.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"/Users/michaelliesenberg/ti/ti_cgt_tiarmclang_1.2.1.STS/bin/tiarmclang" -c -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -mlittle-endian -mthumb -Oz -I"/Users/michaelliesenberg/workspace_CC2652P/bim_onchip_CC1352P_2_LAUNCHXL_nortos_ticlang" -I"/Users/michaelliesenberg/ti/ti_cgt_tiarmclang_1.2.1.STS/include" -I"/Applications/ti/simplelink_cc13xx_cc26xx_sdk_6_40_00_13/source" -I"/Applications/ti/simplelink_cc13xx_cc26xx_sdk_6_40_00_13/source/ti/common/flash/no_rtos/extFlash" -I"/Applications/ti/simplelink_cc13xx_cc26xx_sdk_6_40_00_13/source/ti/devices/cc13x2_cc26x2/startup_files" -DDeviceFamily_CC13X2 -DSET_CCFG_IMAGE_VALID_CONF_IMAGE_VALID=0x56000 -DSET_CCFG_MODE_CONF_XOSC_CAP_MOD=0x0 -DSET_CCFG_MODE_CONF_XOSC_CAPARRAY_DELTA=-63 -DBIM_ONCHIP -DxSECURITY -DDEBUG_BIM -gdwarf-3 -fshort-enums -munaligned-access -funsigned-char -fcommon -ffunction-sections -fdata-sections -march=armv7e-m -MMD -MP -MF"Application/$(basename $(<F)).d_raw" -MT"$(@)" -std=gnu9x $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


