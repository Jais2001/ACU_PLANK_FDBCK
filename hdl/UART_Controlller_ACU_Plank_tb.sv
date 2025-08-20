`timescale 1ns/1ps
module UART_Controlller_ACU_Plank_tb();

    bit i_clk_100 = 0;
    bit i_rst_n_1;
    bit i_rst_n_2;
    logic i_rx_serial;

    bit i_attn_31p5;
    bit i_phase_180;
    bit i_TR_pulse;
    bit i_inhibit;

    logic [7:0] i_fdbck_plank;

    logic[3:0] o_enable;
    logic[5:0] o_ATT1;
    logic[5:0] o_ATT2;
    logic[5:0] o_ATT3;
    logic[5:0] o_ATT4;
    logic o_BITE_CNTRL;
    logic o_SUB_ARRAY;
    logic[7:0] o_tx_serial;
    logic[7:0] o_reset_plank;
    logic o_tx_serial_fdbck;

    logic o_attn_31p5_0;  
    logic o_attn_31p5_1; 
    logic o_attn_31p5_2; 
    logic o_attn_31p5_3; 
    logic o_attn_31p5_4; 
    logic o_attn_31p5_5; 
    logic o_attn_31p5_6; 
    logic o_attn_31p5_7; 
    logic o_phase_180_0; 
    logic o_phase_180_1; 
    logic o_phase_180_2; 
    logic o_phase_180_3; 
    logic o_phase_180_4; 
    logic o_phase_180_5; 
    logic o_phase_180_6; 
    logic o_phase_180_7; 
    logic o_TR_pulse_0; 
    logic o_TR_pulse_1; 
    logic o_TR_pulse_2; 
    logic o_TR_pulse_3; 
    logic o_TR_pulse_4; 
    logic o_TR_pulse_5; 
    logic o_TR_pulse_6; 
    logic o_TR_pulse_7; 
    logic o_inhibit_0; 
    logic o_inhibit_1; 
    logic o_inhibit_2; 
    logic o_inhibit_3; 
    logic o_inhibit_4; 
    logic o_inhibit_5; 
    logic o_inhibit_6; 
    logic o_inhibit_7;

    parameter c_BIT_PERIOD = 8680;

    reg [7:0] ACU_checksum = 8'd0;
    reg [7:0] ACU_data [0:8];

    reg [7:0] SNSR_checksum = 8'd0;
    reg [7:0] SNSR_data [0:4];

    reg [7:0] PLANK_data[0:21];
    reg [7:0] PLANK_checksum = 8'd0;

    integer k,b;

    initial begin
    
        ACU_data[0] = 8'hAA; //header
        ACU_data[8] = 8'h55; // Footer

        SNSR_data[0] = 8'hAA;
        SNSR_data[1] = 8'h04;
        SNSR_data[2] = 8'h11;
        SNSR_data[4] = 8'h55;

        PLANK_data[0] = 8'hAA; //header
        PLANK_data[1] = {3'b111,1'd0,4'h2};
        PLANK_data[21] = 8'h55;// Footer

        for (k = 2;k<19;k = k+ 1) begin
            PLANK_data[k] = {1'd0,1'd0,6'b110010};
        end    

        for (b =0;b<3;b=b+1) begin
            SNSR_checksum = SNSR_checksum ^ SNSR_data[b];
        end

        SNSR_data[3] = SNSR_checksum;
    end


    UART_Controlller_ACU_Plank UART_Controlller_ACU_Plank_inst(
        .i_clk_100(i_clk_100),
        .i_rst_n(i_rst_n_1),
        // .i_usr_rst(i_rst_n_2),
        .i_rx_serial(i_rx_serial),
        .i_attn_31p5(i_attn_31p5),
        .i_phase_180(i_phase_180),
        .i_TR_pulse(i_TR_pulse),
        .i_inhibit(i_inhibit),
        .i_fdbck_plank(i_fdbck_plank),
        .o_enable(o_enable),
        .o_ATT1(o_ATT1),
        .o_ATT2(o_ATT2),
        .o_ATT3(o_ATT3),
        .o_ATT4(o_ATT4),
        .o_BITE_CNTRL(o_BITE_CNTRL),
        .o_SUB_ARRAY(o_SUB_ARRAY),
        .o_tx_serial(o_tx_serial),
        .o_reset_plank(o_reset_plank),
        .o_tx_serial_fdbck(o_tx_serial_fdbck),
        .o_attn_31p5_0(o_attn_31p5_0),  
        .o_attn_31p5_1(o_attn_31p5_1), 
        .o_attn_31p5_2(o_attn_31p5_2), 
        .o_attn_31p5_3(o_attn_31p5_3), 
        .o_attn_31p5_4(o_attn_31p5_4), 
        .o_attn_31p5_5(o_attn_31p5_5), 
        .o_attn_31p5_6(o_attn_31p5_6), 
        .o_attn_31p5_7(o_attn_31p5_7), 
        .o_phase_180_0(o_phase_180_0), 
        .o_phase_180_1(o_phase_180_1), 
        .o_phase_180_2(o_phase_180_2), 
        .o_phase_180_3(o_phase_180_3), 
        .o_phase_180_4(o_phase_180_4), 
        .o_phase_180_5(o_phase_180_5), 
        .o_phase_180_6(o_phase_180_6), 
        .o_phase_180_7(o_phase_180_7), 
        .o_TR_pulse_0(o_TR_pulse_0), 
        .o_TR_pulse_1(o_TR_pulse_1), 
        .o_TR_pulse_2(o_TR_pulse_2), 
        .o_TR_pulse_3(o_TR_pulse_3), 
        .o_TR_pulse_4(o_TR_pulse_4), 
        .o_TR_pulse_5(o_TR_pulse_5), 
        .o_TR_pulse_6(o_TR_pulse_6), 
        .o_TR_pulse_7(o_TR_pulse_7), 
        .o_inhibit_0(o_inhibit_0), 
        .o_inhibit_1(o_inhibit_1), 
        .o_inhibit_2(o_inhibit_2), 
        .o_inhibit_3(o_inhibit_3), 
        .o_inhibit_4(o_inhibit_4), 
        .o_inhibit_5(o_inhibit_5), 
        .o_inhibit_6(o_inhibit_6), 
        .o_inhibit_7(o_inhibit_7)
    );

    PLANK #(.temp(24'h2050DD)) PLANK1_module_inst      // VCC,GND - 32*c 
    (
        .i_clk(i_clk_100),
        .i_rst(i_rst_n_1),
        .i_rx_serial(o_tx_serial[0]),
        .o_tx_serial(i_fdbck_plank[0])
    );

    PLANK #(.temp(24'h1920DD)) PLANK2_module_inst   // VCC,GND - 25*c
    (
        .i_clk(i_clk_100),
        .i_rst(i_rst_n_1),
        .i_rx_serial(o_tx_serial[1]),
        .o_tx_serial(i_fdbck_plank[1])
    );

    PLANK #(.temp(24'h1E22DD)) PLANK3_module_inst  // VCC,GND - 30*c
            (
                .i_clk(i_clk_100),
                .i_rst(i_rst_n_1),
                .i_rx_serial(o_tx_serial[2]),
                .o_tx_serial(i_fdbck_plank[2])
            );

    PLANK #(.temp(24'h282DDD)) PLANK4_module_inst  // VCC,GND - 40*c
    (
        .i_clk(i_clk_100),
        .i_rst(i_rst_n_1),
        .i_rx_serial(o_tx_serial[3]),
        .o_tx_serial(i_fdbck_plank[3])
    );

    PLANK #(.temp(24'h2D90DD)) PLANK5_module_inst  // VCC,GND - 45*c
            (
                .i_clk(i_clk_100),
                .i_rst(i_rst_n_1),
                .i_rx_serial(o_tx_serial[4]),
                .o_tx_serial(i_fdbck_plank[4])
            );

    PLANK #(.temp(24'h5050DD)) PLANK6_module_inst  // VCC,GND - 50*c
    (
        .i_clk(i_clk_100),
        .i_rst(i_rst_n_1),
        .i_rx_serial(o_tx_serial[5]),
        .o_tx_serial(i_fdbck_plank[5])
    );

    PLANK #(.temp(24'h3737DD)) PLANK7_module_inst   // VCC,GND - 55*c
            (
                .i_clk(i_clk_100),
                .i_rst(i_rst_n_1),
                .i_rx_serial(o_tx_serial[6]),
                .o_tx_serial(i_fdbck_plank[6])
            );

    PLANK #(.temp(24'h3C3CDD)) PLANK8_module_inst // VCC,GND - 60*c
    (
        .i_clk(i_clk_100),
        .i_rst(i_rst_n_1),
        .i_rx_serial(o_tx_serial[7]),
        .o_tx_serial(i_fdbck_plank[7])
    );

    task UART_WRITE;
        input[7:0] in_data;
        integer i;
        begin
            i_rx_serial <= 1'b0;
            #(c_BIT_PERIOD);
            for (i=0;i<8;i=i+1) begin
                i_rx_serial = in_data[i];
                #(c_BIT_PERIOD);
            end
            i_rx_serial = 1'b1;
            #(c_BIT_PERIOD);
        end
    endtask 

    task UART_Fedback_WRITE;
        input[7:0] in_data;
        // input[2:0] in_module;
        integer i;
        begin
            i_fdbck_plank[7] <= 1'b0;
            #(c_BIT_PERIOD);
            for (i=0;i<8;i=i+1) begin
                i_fdbck_plank[7] = in_data[i];
                #(c_BIT_PERIOD);
            end
            i_fdbck_plank[7] = 1'b1;
            #(c_BIT_PERIOD);
        end
    endtask

    task  SNSR_FDBCK;
        integer o;
        begin
            foreach(SNSR_data[o])begin
                UART_WRITE(SNSR_data[o]);
            end
        end
    endtask 

    task  set_feedback;
        input[2:0] in_module;
        integer i;
        begin
          foreach(PLANK_data[i])begin
            UART_Fedback_WRITE(PLANK_data[i]);
          end
        end
    endtask 

    task send_ACU;
        integer i,m;
        input [3:0] i_enable;
        input [5:0] i_ATT1;
        input [5:0] i_ATT2;
        input [5:0] i_ATT3;
        input [5:0] i_ATT4;
        input  i_BITE_CNTRL;
        input  i_SUB_ARRAY;
        begin
            ACU_checksum = 8'd0;
            ACU_data[1] = {i_enable[3],i_enable[2],i_enable[1],i_enable[0],4'h1};
            ACU_data[2] = {1'd0,1'd0,i_ATT1};
            ACU_data[3] = {1'd0,1'd0,i_ATT2};
            ACU_data[4] = {1'd0,1'd0,i_ATT3};
            ACU_data[5] = {1'd0,1'd0,i_ATT4};
            ACU_data[6] = {6'd0,i_SUB_ARRAY,i_BITE_CNTRL};
            for (i=0;i<7;i=i+1) begin
                ACU_checksum = ACU_checksum ^ ACU_data[i];
            end
            ACU_data[7] = ACU_checksum;
            foreach(ACU_data[m]) begin
                UART_WRITE(ACU_data[m]);
            end
        end
    endtask 

    task  send_Plank;
        input [2:0] plank_module;
        integer i,m;
        begin 
            PLANK_checksum = 8'd0;
            PLANK_data[19] = plank_module;
            for (i=0;i<20;i=i+1) begin
                PLANK_checksum = PLANK_checksum ^ PLANK_data[i];
            end
            PLANK_data[20] = PLANK_checksum;
            foreach(PLANK_data[m]) begin
                UART_WRITE(PLANK_data[m]);
            end
        end
    endtask 

    always @(UART_Controlller_ACU_Plank_inst.r_uart_tx_data_fdbck) begin
        if (UART_Controlller_ACU_Plank_inst.r_uart_tx_data_fdbck == 8'hee) begin
            $display("Caught ACK at time %t", $time);
        end
    end


    initial begin
        i_rst_n_1 = 0;
        #50;
        i_rst_n_1 = 1;
        #30000;
        SNSR_FDBCK;
        #1_0024240;
        // $display("Time started Change_ACU %t", $time);
        $display("Time started sending feedback %t", $time);
        #3000;
        $display("Time started sending Plank 1 %t", $time);
        send_Plank(3'd0);
        wait(UART_Controlller_ACU_Plank_inst.r_ACK_FDBCK_done);
        #20000;
        $display("Time started sending Plank 2 %t", $time);
        send_Plank(3'd1);
        wait(UART_Controlller_ACU_Plank_inst.r_ACK_FDBCK_done);
        #20000;
        $display("Time started sending Plank 3 %t", $time);
        send_Plank(3'd2);
        wait(UART_Controlller_ACU_Plank_inst.r_ACK_FDBCK_done);
        #20000;
        $display("Time started sending Plank 4 %t", $time);
        send_Plank(3'd3);
        wait(UART_Controlller_ACU_Plank_inst.r_ACK_FDBCK_done);
        #20000;
        $display("Time started sending Plank 5 %t", $time);
        send_Plank(3'd4);
        wait(UART_Controlller_ACU_Plank_inst.r_ACK_FDBCK_done);
        #20000;
        $display("Time started sending Plank 6 %t", $time);
        send_Plank(3'd5);
        wait(UART_Controlller_ACU_Plank_inst.r_ACK_FDBCK_done);
        #20000;
        SNSR_FDBCK;
        #3_0024240;
        $stop;
        // set_feedback(3'd0);
        // #100;
        // set_feedback(3'd7);
        // #100;
        // set_feedback(3'd2);
        // #100;
        // set_feedback(3'd3);
        // #100;
        // set_feedback(3'd4);
        // #100;
        // set_feedback(3'd5);
        // #100;
        // set_feedback(3'd6);
        // #100;
        // set_feedback(3'd7);
        // #100;
        // $stop;
        // send_ACU(4'hf,6'b110011,6'b101010,6'b111000,6'b100100,1'b1,1'b1);
        // $display("Time stoped Changed_ACU %t", $time);
        // #10;
        // $display("Time started Change_PLANK %t", $time);
        // send_Plank;
        // #100;
        // $display("Time stoped Change_PLANK %t", $time);
        // $stop;
    end

    always
        #5 i_clk_100 = ~i_clk_100;


endmodule