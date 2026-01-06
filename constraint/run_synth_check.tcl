set_device -family {SmartFusion2} -die {M2S090TS} -speed {STD}
set_editor_type {SYNTHESIS}
set_proj_path {D:\RFMW\APASS\ACU_PLANK_HDL\ACU_PLANK_FDBCK\UART_Controlller_ACU_Plank.prjx}
read_verilog -mode system_verilog {D:\RFMW\APASS\ACU_PLANK_HDL\ACU_PLANK_FDBCK\component\work\FCCC_C4\FCCC_C4_0\FCCC_C4_FCCC_C4_0_FCCC.v}
read_verilog -mode system_verilog {D:\RFMW\APASS\ACU_PLANK_HDL\ACU_PLANK_FDBCK\component\work\FCCC_C4\FCCC_C4.v}
read_verilog -mode system_verilog {D:\RFMW\APASS\ACU_PLANK_HDL\ACU_PLANK_FDBCK\component\work\OSC_C4\OSC_C4_0\OSC_C4_OSC_C4_0_OSC.v}
read_verilog -mode system_verilog {D:\RFMW\APASS\ACU_PLANK_HDL\ACU_PLANK_FDBCK\component\work\OSC_C4\OSC_C4.v}
read_verilog -mode system_verilog {D:\RFMW\APASS\ACU_PLANK_HDL\ACU_PLANK_FDBCK\hdl\UART_Packet_Identifier.v}
read_verilog -mode system_verilog {D:\RFMW\APASS\ACU_PLANK_HDL\ACU_PLANK_FDBCK\hdl\UART_rx.v}
read_verilog -mode system_verilog {D:\RFMW\APASS\ACU_PLANK_HDL\ACU_PLANK_FDBCK\hdl\UART_tx.v}
read_verilog -mode system_verilog {D:\RFMW\APASS\ACU_PLANK_HDL\ACU_PLANK_FDBCK\hdl\UART_Controlller_ACU_Plank.v}
read_verilog -mode system_verilog {D:\RFMW\APASS\ACU_PLANK_HDL\ACU_PLANK_FDBCK\component\work\ACU_PLANK\ACU_PLANK.v}
set_top_level {ACU_PLANK}
map_netlist
set_output_sdc {D:\RFMW\APASS\ACU_PLANK_HDL\ACU_PLANK_FDBCK\constraint\user.sdc}
