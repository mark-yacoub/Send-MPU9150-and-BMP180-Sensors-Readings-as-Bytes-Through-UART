################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
drivers/rgb.obj: C:/ti/TivaWare_C_Series-2.0.1.11577/examples/boards/ek-tm4c123gxl-boostxl-senshub/drivers/rgb.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"c:/ti/ccsv6/tools/compiler/arm_5.1.11/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 --abi=eabi -me -O2 -g --include_path="c:/ti/ccsv6/tools/compiler/arm_5.1.11/include" --include_path="C:/ti/TivaWare_C_Series-2.0.1.11577/examples/boards/ek-tm4c123gxl-boostxl-senshub" --include_path="C:/ti/TivaWare_C_Series-2.0.1.11577" --gcc --define=ccs="ccs" --define=PART_TM4C123GH6PM --define=TARGET_IS_BLIZZARD_RB1 --define=UART_BUFFERED --diag_warning=225 --display_error_number --diag_wrap=off --gen_func_subsections=on --ual --preproc_with_compile --preproc_dependency="drivers/rgb.pp" --obj_directory="drivers" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

