# THIS FILE IS AUTOMATICALLY GENERATED
# Project: C:\Users\prichards\proj\acsnb-motor-cypress\QuadEncoderRTOS.cydsn\QuadEncoderRTOS.cyprj
# Date: Tue, 28 Feb 2023 01:33:05 GMT
#set_units -time ns
create_clock -name {I2C_SCBCLK(FFB)} -period 625 -waveform {0 312.5} [list [get_pins {ClockBlock/ff_div_2}]]
create_clock -name {Clock_5kHz(FFB)} -period 200000 -waveform {0 100000} [list [get_pins {ClockBlock/ff_div_8}]]
create_clock -name {Clock_1(FFB)} -period 41.666666666666664 -waveform {0 20.8333333333333} [list [get_pins {ClockBlock/ff_div_10}]]
create_clock -name {SPI_1_SCBCLK(FFB)} -period 20.833333333333332 -waveform {0 10.4166666666667} [list [get_pins {ClockBlock/ff_div_3}]]
create_clock -name {CyRouted1} -period 20.833333333333332 -waveform {0 10.4166666666667} [list [get_pins {ClockBlock/dsi_in_0}]]
create_clock -name {CyILO} -period 31250 -waveform {0 15625} [list [get_pins {ClockBlock/ilo}]]
create_clock -name {CyLFClk} -period 31250 -waveform {0 15625} [list [get_pins {ClockBlock/lfclk}]]
create_clock -name {CyIMO} -period 20.833333333333332 -waveform {0 10.4166666666667} [list [get_pins {ClockBlock/imo}]]
create_clock -name {CyHFClk} -period 20.833333333333332 -waveform {0 10.4166666666667} [list [get_pins {ClockBlock/hfclk}]]
create_clock -name {CySysClk} -period 20.833333333333332 -waveform {0 10.4166666666667} [list [get_pins {ClockBlock/sysclk}]]
create_generated_clock -name {I2C_SCBCLK} -source [get_pins {ClockBlock/hfclk}] -edges {1 31 61} [list]
create_generated_clock -name {Clock_5kHz} -source [get_pins {ClockBlock/hfclk}] -edges {1 9601 19201} [list]
create_generated_clock -name {Clock_1} -source [get_pins {ClockBlock/hfclk}] -edges {1 3 5} [list [get_pins {ClockBlock/udb_div_0}]]
create_generated_clock -name {SPI_1_SCBCLK} -source [get_pins {ClockBlock/hfclk}] -edges {1 2 3} [list]

set_false_path -from [get_pins {__ONE__/q}]

# Component constraints for C:\Users\prichards\proj\acsnb-motor-cypress\QuadEncoderRTOS.cydsn\TopDesign\TopDesign.cysch
# Project: C:\Users\prichards\proj\acsnb-motor-cypress\QuadEncoderRTOS.cydsn\QuadEncoderRTOS.cyprj
# Date: Tue, 28 Feb 2023 01:33:02 GMT
