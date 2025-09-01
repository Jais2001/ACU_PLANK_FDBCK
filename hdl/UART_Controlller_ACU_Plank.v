module UART_Controlller_ACU_Plank (
    input wire i_clk_100,
    input wire i_rst_n,
    input wire i_usr_rst,
    input wire i_rx_serial,

    input wire i_attn_31p5,
    input wire i_phase_180,
    input wire i_TR_pulse,
    input wire i_inhibit,

    input wire[7:0] i_fdbck_plank,

    output wire[3:0] o_enable,
    output wire[5:0] o_ATT1,
    output wire[5:0] o_ATT2,
    output wire[5:0] o_ATT3,
    output wire[5:0] o_ATT4,

    output wire o_BITE_CNTRL,
    output wire o_SUB_ARRAY,

    output wire[7:0] o_reset_plank,
    output wire[7:0] o_tx_serial,
    output wire o_tx_serial_fdbck,

    output wire o_attn_31p5_0,    
    output wire o_attn_31p5_1, 
    output wire o_attn_31p5_2, 
    output wire o_attn_31p5_3, 
    output wire o_attn_31p5_4, 
    output wire o_attn_31p5_5, 
    output wire o_attn_31p5_6, 
    output wire o_attn_31p5_7, 

    output wire o_phase_180_0, 
    output wire o_phase_180_1, 
    output wire o_phase_180_2, 
    output wire o_phase_180_3, 
    output wire o_phase_180_4, 
    output wire o_phase_180_5, 
    output wire o_phase_180_6, 
    output wire o_phase_180_7, 

    output wire o_TR_pulse_0, 
    output wire o_TR_pulse_1, 
    output wire o_TR_pulse_2, 
    output wire o_TR_pulse_3, 
    output wire o_TR_pulse_4, 
    output wire o_TR_pulse_5, 
    output wire o_TR_pulse_6, 
    output wire o_TR_pulse_7, 

    output wire o_inhibit_0, 
    output wire o_inhibit_1, 
    output wire o_inhibit_2, 
    output wire o_inhibit_3, 
    output wire o_inhibit_4, 
    output wire o_inhibit_5, 
    output wire o_inhibit_6, 
    output wire o_inhibit_7 


);
    localparam CLOCK_FREQ = 100000000;
    localparam BAUD_RATE = 115200;
    localparam CLKS_PER_BIT = CLOCK_FREQ/BAUD_RATE;

    wire w_rst_n;
    
    wire [7:0] w_uart_rx_data;
    wire w_uart_rx_valid;

    reg[7:0] r_uart_rx_data;
    reg r_uart_rx_valid;

    reg[3:0] r_enable;

    reg r_soft_inhibit;

    reg r_uart_tx_valid_fdbck;
    reg [7:0] r_uart_tx_data_fdbck;

    reg [5:0] r_ATT1;
    reg [5:0] r_ATT2;
    reg [5:0] r_ATT3;
    reg [5:0] r_ATT4;
    reg  r_BITE_CNTRL;
    reg  r_SUB_ARRAY;
    reg [2:0] r_Plank_module;
    reg[7:0] r_buff_Plank_module;

    reg[6:0]r_ACK_send;

    reg[151:0] r_send_plank;
    reg[4:0] r_send_counter;
    reg[151:0] r_plank_data;

    wire w_uart_tx_active;
    reg r_uart_tx_valid = 0;
    reg [7:0] r_uart_tx_data = 0;
    wire w_uart_tx_done;
    wire w_tx_serial;

    wire w_uart_tx_active_fdbck;
    wire w_uart_tx_done_fdbck;

    wire [7:0] w_checksum;

    reg r_PLANK_data_valid_buff_1;
    reg r_PLANK_data_valid_buff_2;
    reg r_PLANK_data_valid_buff_3;
    // reg r_PLANK_data_valid_buff_28;

    wire w_rst_PLANK;

    wire [7:0]w_rx_serial_fdbck_Plank;
    wire [7:0] w_uart_rx_data_fdbck_Plank[7:0];
    wire [7:0]w_uart_rx_valid_fdbck_Plank;

    reg[7:0] r_plank_data_module[7:0];
    reg [7:0]r_plank_module_valid;

    reg r_ACK_plank_module_valid; 
    reg[7:0] r_ACK_plank_data_module;  

    reg[29:0] r_shift_PLANK_valid;

    uart_rx #(.CLKS_PER_BIT(CLKS_PER_BIT)) uart_rx_inst
    (
        .i_Clock (i_clk_100),
        .i_Rx_Serial (i_rx_serial), 
        .i_rst (w_rst_n) , 
        .o_Rx_DV (w_uart_rx_valid),
        .o_Rx_Byte (w_uart_rx_data)
    );

    uart_tx #(.CLKS_PER_BIT(CLKS_PER_BIT)) uart_tx_inst 
    (
        .i_Clock (i_clk_100),
        .i_Tx_DV (r_uart_tx_valid),
        .i_rst (w_rst_n) , 
        .i_Tx_Byte (r_uart_tx_data),
        .o_Tx_Active (w_uart_tx_active),
        .o_Tx_Serial (w_tx_serial),
        .o_Tx_Done (w_uart_tx_done)
    );

    genvar i;
    generate
        for (i = 0; i < 8; i = i + 1) begin : gen_uart_rx_fdbck
            uart_rx #(.CLKS_PER_BIT(CLKS_PER_BIT)) uart_rx_inst_fdbck
            (
                .i_Clock (i_clk_100),
                .i_Rx_Serial (w_rx_serial_fdbck_Plank[i]),
                .i_rst (w_rst_n) , 
                .o_Rx_DV (w_uart_rx_valid_fdbck_Plank[i]),
                .o_Rx_Byte (w_uart_rx_data_fdbck_Plank[i])
            );
        end
    endgenerate


    uart_tx #(.CLKS_PER_BIT(CLKS_PER_BIT)) uart_tx_inst_fdbck 
    (
        .i_Clock (i_clk_100),
        .i_Tx_DV (r_uart_tx_valid_fdbck),
        .i_rst (w_rst_n) , 
        .i_Tx_Byte (r_uart_tx_data_fdbck),
        .o_Tx_Active (w_uart_tx_active_fdbck),
        .o_Tx_Serial (o_tx_serial_fdbck),
        .o_Tx_Done (w_uart_tx_done_fdbck)
    );

    always @(posedge i_clk_100 or negedge w_rst_n) begin
        if (w_rst_n == 1'b0) begin
            r_uart_rx_data <= 0;
            r_uart_rx_valid <= 0;
        end else begin
            r_uart_rx_data <= w_uart_rx_data;
            r_uart_rx_valid <= w_uart_rx_valid;
        end
    end

    assign w_rst_n = ~i_rst_n ^ i_usr_rst;
    // assign w_rst_n = i_rst_n;

    // Variables for the ACU packet identifier
    localparam ACU_LENGTH = 7; // Total packet size In bytes inluding Identifier,data,checksum
    wire [(ACU_LENGTH*8) - 1 : 0] w_ACU_data;
    wire w_ACU_data_valid;

    // ACU packet identifier
    UART_packet_identifier #(
        .RX_PACKET_LEN(ACU_LENGTH),
        .IDENTIFIER(4'h1)
    ) read_ACU_id_inst(
        .i_clk(i_clk_100),
        .i_rst_n(w_rst_n),
        .i_en(1'b1),
        .i_uart_rx_data(r_uart_rx_data),
        .o_uart_rx_error(),
        .i_uart_rx_valid(r_uart_rx_valid),
        .o_data(w_ACU_data),
        .o_data_valid(w_ACU_data_valid)
    );

    // Variables for the PLANK packet identifier
    localparam PLANK_LENGTH = 20; // Total packet size In bytes inluding Identifier,data,checksum
    wire [(PLANK_LENGTH*8) - 1 : 0] w_PLANK_data;
    wire w_PLANK_data_valid;

    // PLANk packet identifier
    UART_packet_identifier #(
        .RX_PACKET_LEN(PLANK_LENGTH),
        .IDENTIFIER(4'h2)
    ) read_PLANK_id_inst(
        .i_clk(i_clk_100),
        .i_rst_n(w_rst_n),
        .i_en(1'b1),
        .i_uart_rx_data(r_uart_rx_data),
        .o_uart_rx_error(),
        .i_uart_rx_valid(r_uart_rx_valid),
        .o_data(w_PLANK_data),
        .o_data_valid(w_PLANK_data_valid)
    );


    // Variables for Sensor Readback identifier
    localparam SNSR_LENGTH = 3; // Total packet size In bytes inluding Identifier,data,checksum
    wire [(SNSR_LENGTH*8) - 1 : 0] w_SNSR_data;
    wire w_SNSR_data_valid;

    // Sensor Readback identifier
    UART_packet_identifier #(
        .RX_PACKET_LEN(SNSR_LENGTH),
        .IDENTIFIER(4'h4)
    ) SNSR_RDBCK_inst(
        .i_clk(i_clk_100),
        .i_rst_n(w_rst_n),
        .i_en(1'b1),
        .i_uart_rx_data(r_uart_rx_data),
        .o_uart_rx_error(),
        .i_uart_rx_valid(r_uart_rx_valid),
        .o_data(w_SNSR_data),
        .o_data_valid(w_SNSR_data_valid)
    );

    always @(posedge i_clk_100 or negedge w_rst_n) begin
            if(w_rst_n == 1'b0) begin 
                r_enable <= 0;
                r_ATT1  <= 0;
                r_ATT2  <= 0;
                r_ATT3  <= 0;
                r_ATT4  <= 0;
                r_BITE_CNTRL <= 0;
                r_SUB_ARRAY <= 0;
            end
            else begin
                if(w_ACU_data_valid) begin
                    r_enable[0]     <= w_ACU_data[4];
                    r_enable[1]     <= w_ACU_data[5];
                    r_enable[2]     <= w_ACU_data[6];
                    r_enable[3]     <= w_ACU_data[7];
                    r_ATT1          <= w_ACU_data[13:8];
                    r_ATT2          <= w_ACU_data[21:16];
                    r_ATT3          <= w_ACU_data[29:24];
                    r_ATT4          <= w_ACU_data[37:32];
                    r_BITE_CNTRL    <= w_ACU_data[40];
                    r_SUB_ARRAY     <= w_ACU_data[41];
            end
        end
    end

    always @(posedge i_clk_100 or negedge w_rst_n) begin
        if (~w_rst_n) begin
            r_Plank_module <= 0;
            r_plank_data   <= 0;
            r_soft_inhibit <= 0;
        end else begin
            if (w_PLANK_data_valid) begin 
                r_soft_inhibit       <= w_PLANK_data[7];
                r_plank_data    <= {w_PLANK_data[143:8],4'b0000,w_PLANK_data[7:6],2'b00,8'hAA};
                r_Plank_module  <= w_PLANK_data[146:144];                                 
            end 
        end
    end

    always @(posedge i_clk_100) begin
        r_buff_Plank_module <= r_Plank_module;
    end

    always @(posedge i_clk_100 or negedge w_rst_n) begin // for getting the delayed PLANK data valid signal
        if (~w_rst_n) begin
            r_shift_PLANK_valid <= 30'd0;
        end else begin
            r_shift_PLANK_valid <= (r_shift_PLANK_valid << 1) | w_PLANK_data_valid;
        end
    end

    assign r_PLANK_data_valid_buff_1 = r_shift_PLANK_valid[0];
    assign r_PLANK_data_valid_buff_2 = r_shift_PLANK_valid[1];
    assign r_PLANK_data_valid_buff_3 = r_shift_PLANK_valid[2];
    // assign r_PLANK_data_valid_buff_28 = r_shift_PLANK_valid[27]; // 28th delayed cycle for getting ACK from PLANK

    assign w_checksum = r_plank_data[15:8]  ^ r_plank_data[23:16] ^
                        r_plank_data[31:24] ^ r_plank_data[39:32] ^ r_plank_data[47:40] ^
                        r_plank_data[55:48] ^ r_plank_data[63:56] ^ r_plank_data[71:64] ^
                        r_plank_data[79:72] ^ r_plank_data[87:80] ^ r_plank_data[95:88] ^
                        r_plank_data[103:96] ^ r_plank_data[111:104] ^ r_plank_data[119:112] ^
                        r_plank_data[127:120] ^ r_plank_data[135:128] ^ r_plank_data[143:136] ^
                        r_plank_data[151:144];

    assign w_rst_PLANK = w_PLANK_data[5];

    reg[2:0] send_plank_data;
    localparam SM_1 = 3'd0;
    localparam SM_2 = 3'd1;
    localparam SM_3 = 3'd2;
    localparam SM_4 = 3'd3;

    reg r_ACK_FDBCK_done;
    reg r_buff_ACK_FDBCK_done;

    reg[1:0] SM_ACK_FDBCK;
    localparam SM_Fdbck_ACK_1 = 3'd0;
    localparam SM_Fdbck_ACK_2 = 3'd1;
    localparam SM_Fdbck_ACK_3 = 3'd2;
    localparam SM_Fdbck_ACK_4 = 3'd3;

    reg[2:0] SM_Sensor_Fdcbk;
    localparam SM_Sensor_wait = 3'd0;
    localparam SM_Sensor_1 = 3'd1;
    localparam SM_INCT_Cnt = 3'd2;
    localparam SM_SNSR_STRT = 3'd3;
    localparam SM_PLNK_INCRT = 3'd4;
    localparam SM_Delay_State = 3'd5;

    always @(posedge i_clk_100 or negedge w_rst_n) begin // buffering 
        if (~w_rst_n) begin
            r_plank_module_valid <= 1'd0;
            // r_plank_data_module        <= 8'd0;
            r_ACK_plank_module_valid   <= 1'd0;
            r_ACK_plank_data_module   <= 8'd0; 
        end else begin
            // if (r_ACK_FDBCK_done) begin
            //     r_ACK_plank_module_valid <= w_uart_rx_valid_fdbck_Plank[r_Plank_module];
            //     r_ACK_plank_data_module  <= w_uart_rx_data_fdbck_Plank[r_Plank_module];
            // end else begin
                r_plank_module_valid[0] <= w_uart_rx_valid_fdbck_Plank[0];
                r_plank_data_module[0]  <= w_uart_rx_data_fdbck_Plank[0];

                r_plank_module_valid[1] <= w_uart_rx_valid_fdbck_Plank[1];
                r_plank_data_module[1]  <= w_uart_rx_data_fdbck_Plank[1];

                r_plank_module_valid[2] <= w_uart_rx_valid_fdbck_Plank[2];
                r_plank_data_module[2]  <= w_uart_rx_data_fdbck_Plank[2];

                r_plank_module_valid[3] <= w_uart_rx_valid_fdbck_Plank[3];
                r_plank_data_module[3]  <= w_uart_rx_data_fdbck_Plank[3];

                r_plank_module_valid[4] <= w_uart_rx_valid_fdbck_Plank[4];
                r_plank_data_module[4]  <= w_uart_rx_data_fdbck_Plank[4];

                r_plank_module_valid[5] <= w_uart_rx_valid_fdbck_Plank[5];
                r_plank_data_module[5]  <= w_uart_rx_data_fdbck_Plank[5];

                r_plank_module_valid[6] <= w_uart_rx_valid_fdbck_Plank[6];
                r_plank_data_module[6]  <= w_uart_rx_data_fdbck_Plank[6];

                r_plank_module_valid[7] <= w_uart_rx_valid_fdbck_Plank[7];
                r_plank_data_module[7]  <= w_uart_rx_data_fdbck_Plank[7];       
            // end
        end
    end

    reg r_plank1_done;
    reg r_plank2_done;
    reg r_plank3_done;
    reg r_plank4_done;
    reg r_plank5_done;
    reg r_plank6_done;
    reg r_plank7_done;
    reg r_plank8_done;

    reg r_plank1_pending_done;
    reg r_plank2_pending_done;
    reg r_plank3_pending_done;
    reg r_plank4_pending_done;
    reg r_plank5_pending_done;
    reg r_plank6_pending_done;
    reg r_plank7_pending_done;
    reg r_plank8_pending_done;

    reg[4:0] r_plank1_cnt;
    reg[4:0] r_plank2_cnt;
    reg[4:0] r_plank3_cnt;
    reg[4:0] r_plank4_cnt;
    reg[4:0] r_plank5_cnt;
    reg[4:0] r_plank6_cnt;
    reg[4:0] r_plank7_cnt;
    reg[4:0] r_plank8_cnt;

    reg[103:0]r_PLANK1_SNSR_data;
    reg[103:0]r_PLANK2_SNSR_data;
    reg[103:0]r_PLANK3_SNSR_data;
    reg[103:0]r_PLANK4_SNSR_data;
    reg[103:0]r_PLANK5_SNSR_data;
    reg[103:0]r_PLANK6_SNSR_data;
    reg[103:0]r_PLANK7_SNSR_data;
    reg[103:0]r_PLANK8_SNSR_data;

    reg[903:0] r_PLNK_all_data;
    reg[903:0] r_PLNK_buff_all_data;


    reg[3:0] RDBCK_Pl1;
    localparam PL1_RFRSH_1 = 4'd0;
    localparam PL1_SM2 = 4'd1;
    localparam PL1_SM3 = 4'd2;
    localparam PL1_RFRSH_2 = 4'd3;

    reg[3:0] RDBCK_Pl2;
    localparam PL2_RFRSH_1 = 4'd0;
    localparam PL2_SM2 = 4'd1;
    localparam PL2_SM3 = 4'd2;
    localparam PL2_RFRSH_2 = 4'd3;

    reg[3:0] RDBCK_Pl3;
    localparam PL3_RFRSH_1 = 4'd0;
    localparam PL3_SM2 = 4'd1;
    localparam PL3_SM3 = 4'd2;
    localparam PL3_RFRSH_2 = 4'd3;

    reg[3:0] RDBCK_Pl4;
    localparam PL4_RFRSH_1 = 4'd0;
    localparam PL4_SM2 = 4'd1;
    localparam PL4_SM3 = 4'd2;
    localparam PL4_RFRSH_2 = 4'd3;

    reg[3:0] RDBCK_Pl5;
    localparam PL5_RFRSH_1 = 4'd0;
    localparam PL5_SM2 = 4'd1;
    localparam PL5_SM3 = 4'd2;
    localparam PL5_RFRSH_2 = 4'd3;

    reg[3:0] RDBCK_Pl6;
    localparam PL6_RFRSH_1 = 4'd0;
    localparam PL6_SM2 = 4'd1;
    localparam PL6_SM3 = 4'd2;
    localparam PL6_RFRSH_2 = 4'd3;

    reg[3:0] RDBCK_Pl7;
    localparam PL7_RFRSH_1 = 4'd0;
    localparam PL7_SM2 = 4'd1;
    localparam PL7_SM3 = 4'd2;
    localparam PL7_RFRSH_2 = 4'd3;

    reg[3:0] RDBCK_Pl8;
    localparam PL8_RFRSH_1 = 4'd0;
    localparam PL8_SM2 = 4'd1;
    localparam PL8_SM3 = 4'd2;
    localparam PL8_RFRSH_2 = 4'd3;

    reg[31:0] PL_1_Time_out;
    reg[31:0] PL_2_Time_out;
    reg[31:0] PL_3_Time_out;
    reg[31:0] PL_4_Time_out;
    reg[31:0] PL_5_Time_out;
    reg[31:0] PL_6_Time_out;
    reg[31:0] PL_7_Time_out;
    reg[31:0] PL_8_Time_out;

    always @(posedge i_clk_100 or negedge w_rst_n) begin
        if (~w_rst_n) begin
            r_plank1_done <= 1'b0;
            r_PLANK1_SNSR_data <= 0;
            PL_1_Time_out <= 0;
            r_plank1_cnt <= 0;
            RDBCK_Pl1 <= PL1_RFRSH_1;
        end else begin
            r_plank1_done <= 1'b0;

            case (RDBCK_Pl1)
                PL1_RFRSH_1: begin
                    RDBCK_Pl1 <= PL1_RFRSH_1;
                    if (w_SNSR_data_valid) begin
                        RDBCK_Pl1 <= PL1_RFRSH_2;
                    end
                end
                PL1_RFRSH_2 : begin
                    PL_1_Time_out <= PL_1_Time_out + 1'b1;
                    if (PL_1_Time_out > 32'd1110000) begin
                        RDBCK_Pl1 <= PL1_SM3;
                        r_PLANK1_SNSR_data <= 0;
                    end
                    if ((r_plank_data_module[0] == 8'hCC) && r_plank_module_valid[0]) begin
                        RDBCK_Pl1 <= PL1_SM2;
                        PL_1_Time_out <= 0;
                        r_PLANK1_SNSR_data <= 0;
                    end 
                end
                PL1_SM2 : begin
                    if (r_plank_module_valid[0])begin
                        r_PLANK1_SNSR_data <= (r_PLANK1_SNSR_data << 8) | r_plank_data_module[0];
                        r_plank1_cnt <= r_plank1_cnt + 1'b1;
                    end 
                    if (r_plank1_cnt < 5'd13) begin
                        RDBCK_Pl1 <= PL1_SM2;
                    end else begin
                        RDBCK_Pl1 <= PL1_SM3;
                        r_plank1_cnt <= 5'd0;
                    end
                end 
                PL1_SM3 : begin
                    r_plank1_done <= 1'b1;
                    RDBCK_Pl1 <= PL1_RFRSH_1;
                    PL_1_Time_out <= 0;
                end
                default: begin
                    RDBCK_Pl1 <= PL1_RFRSH_1;
                end
            endcase
        end
    end    

    always @(posedge i_clk_100 or negedge w_rst_n) begin
        if (~w_rst_n) begin
            r_plank2_done <= 1'b0;
            r_PLANK2_SNSR_data <= 0;
            r_plank2_cnt <= 0;
            PL_2_Time_out <= 0;
            RDBCK_Pl2 <= PL2_RFRSH_1;
        end else begin
            r_plank2_done <= 1'b0;

            case (RDBCK_Pl2)
                PL2_RFRSH_1: begin
                    RDBCK_Pl2 <= PL2_RFRSH_1;
                    if (w_SNSR_data_valid) begin
                        RDBCK_Pl2 <= PL2_RFRSH_2;
                    end
                end
                PL2_RFRSH_2 : begin
                    PL_2_Time_out <= PL_2_Time_out + 1'b1;
                    if (PL_2_Time_out > 32'd1110000) begin
                        RDBCK_Pl2 <= PL2_SM3;
                        r_PLANK2_SNSR_data <= 0;
                    end
                    if ((r_plank_data_module[1] == 8'hCC) && r_plank_module_valid[1]) begin
                        RDBCK_Pl2 <= PL2_SM2;
                        PL_2_Time_out <= 0;
                        r_PLANK2_SNSR_data <= 0;
                    end 
                end
                PL2_SM2 : begin
                    if (r_plank_module_valid[1])begin
                        r_PLANK2_SNSR_data <= (r_PLANK2_SNSR_data << 8) | r_plank_data_module[1];
                        r_plank2_cnt <= r_plank2_cnt + 1'b1;
                    end 
                    if (r_plank2_cnt < 5'd13) begin
                        RDBCK_Pl2 <= PL2_SM2;
                    end else begin
                        RDBCK_Pl2 <= PL2_SM3;
                        r_plank2_cnt <= 5'd0;
                    end
                end 
                PL2_SM3 : begin
                    r_plank2_done <= 1'b1;
                    PL_2_Time_out <= 0;
                    RDBCK_Pl2 <= PL2_RFRSH_1;
                end
                default: begin
                    RDBCK_Pl2 <= PL2_RFRSH_1;
                end
            endcase
        end
    end    


    always @(posedge i_clk_100 or negedge w_rst_n) begin
        if (~w_rst_n) begin
            r_plank3_done <= 1'b0;
            r_PLANK3_SNSR_data <= 0;
            r_plank3_cnt <= 0;
            PL_3_Time_out <= 0;
            RDBCK_Pl3 <= PL3_RFRSH_1;
        end else begin
            r_plank3_done <= 1'b0;

            case (RDBCK_Pl3)
                PL3_RFRSH_1: begin
                    RDBCK_Pl3 <= PL3_RFRSH_1;
                    if (w_SNSR_data_valid) begin
                        RDBCK_Pl3 <= PL3_RFRSH_2;
                    end
                end
                PL3_RFRSH_2 : begin
                    PL_3_Time_out <= PL_3_Time_out + 1'b1;

                    if (PL_3_Time_out > 32'd1110000) begin
                        RDBCK_Pl3 <= PL3_SM3;
                        r_PLANK3_SNSR_data <= 0;
                    end

                    if ((r_plank_data_module[2] == 8'hCC) && r_plank_module_valid[2]) begin
                        RDBCK_Pl3 <= PL3_SM2;
                        r_PLANK3_SNSR_data <= 0;
                        PL_3_Time_out <= 0;
                    end 
                end
                PL3_SM2 : begin
                    if (r_plank_module_valid[2])begin
                        r_PLANK3_SNSR_data <= (r_PLANK3_SNSR_data << 8) | r_plank_data_module[2];
                        r_plank3_cnt <= r_plank3_cnt + 1'b1;
                    end 
                    if (r_plank3_cnt < 5'd13) begin
                        RDBCK_Pl3 <= PL3_SM2;
                    end else begin
                        RDBCK_Pl3 <= PL3_SM3;
                        r_plank3_cnt <= 5'd0;
                    end
                end 
                PL3_SM3 : begin
                    r_plank3_done <= 1'b1;
                    PL_3_Time_out <= 0;
                    RDBCK_Pl3 <= PL3_RFRSH_1;
                end
                default: begin
                    RDBCK_Pl3 <= PL3_RFRSH_1;
                end
            endcase
        end
    end    

    always @(posedge i_clk_100 or negedge w_rst_n) begin
        if (~w_rst_n) begin
            r_plank4_done <= 1'b0;
            r_PLANK4_SNSR_data <= 0;
            r_plank4_cnt <= 0;
            PL_4_Time_out <= 0;
            RDBCK_Pl4 <= PL4_RFRSH_1;
        end else begin
            r_plank4_done <= 1'b0;
            case (RDBCK_Pl4)
                PL4_RFRSH_1 : begin
                    RDBCK_Pl4 <= PL4_RFRSH_1;
                    if (w_SNSR_data_valid) begin
                        RDBCK_Pl4 <= PL4_RFRSH_2;
                    end
                end
                PL4_RFRSH_2 : begin
                    PL_4_Time_out <= PL_4_Time_out + 1'b1;

                    if (PL_4_Time_out > 32'd1110000) begin
                        RDBCK_Pl4 <= PL4_SM3;
                        r_PLANK4_SNSR_data <= 0;
                    end

                    if ((r_plank_data_module[3] == 8'hCC) && r_plank_module_valid[3]) begin
                        PL_4_Time_out <= 0;
                        r_PLANK4_SNSR_data <= 0;
                        RDBCK_Pl4 <= PL4_SM2;
                    end 
                end
                PL4_SM2 : begin
                    if (r_plank_module_valid[3])begin
                        r_PLANK4_SNSR_data <= (r_PLANK4_SNSR_data << 8) | r_plank_data_module[3];
                        r_plank4_cnt <= r_plank4_cnt + 1'b1;
                    end 
                    if (r_plank4_cnt < 5'd13) begin
                        RDBCK_Pl4 <= PL4_SM2;
                    end else begin
                        RDBCK_Pl4 <= PL4_SM3;
                        r_plank4_cnt <= 5'd0;
                    end
                end 
                PL4_SM3 : begin
                    r_plank4_done <= 1'b1;
                    RDBCK_Pl4 <= PL4_RFRSH_1;
                    PL_4_Time_out <= 0;
                end
                default: begin
                    RDBCK_Pl4 <= PL4_RFRSH_1;
                end
            endcase
        end
    end    

    always @(posedge i_clk_100 or negedge w_rst_n) begin
        if (~w_rst_n) begin
            r_plank5_done <= 1'b0;
            r_PLANK5_SNSR_data <= 0;
            r_plank5_cnt <= 0;
            PL_5_Time_out <= 0;
            RDBCK_Pl5 <= PL5_RFRSH_1;
        end else begin
            r_plank5_done <= 1'b0;
            case (RDBCK_Pl5)
                PL5_RFRSH_1 : begin
                    RDBCK_Pl5 <= PL5_RFRSH_1;
                    if (w_SNSR_data_valid) begin
                        RDBCK_Pl5 <= PL5_RFRSH_2;
                    end
                end
                PL5_RFRSH_2 : begin
                    PL_5_Time_out <= PL_5_Time_out + 1'b1;

                    if (PL_5_Time_out > 32'd1110000) begin
                        RDBCK_Pl5 <= PL5_SM3;
                        r_PLANK5_SNSR_data <= 0;
                    end

                    if ((r_plank_data_module[4] == 8'hCC) && r_plank_module_valid[4]) begin
                        PL_5_Time_out <= 0;
                        r_PLANK5_SNSR_data <= 0;
                        RDBCK_Pl5 <= PL5_SM2;
                    end 
                end
                PL5_SM2 : begin
                    if (r_plank_module_valid[4])begin
                        r_PLANK5_SNSR_data <= (r_PLANK5_SNSR_data << 8) | r_plank_data_module[4];
                        r_plank5_cnt <= r_plank5_cnt + 1'b1;
                    end 
                    if (r_plank5_cnt < 5'd13) begin
                        RDBCK_Pl5 <= PL5_SM2;
                    end else begin
                        RDBCK_Pl5 <= PL5_SM3;
                        r_plank5_cnt <= 5'd0;
                    end
                end 
                PL5_SM3 : begin
                    r_plank5_done <= 1'b1;
                    RDBCK_Pl5 <= PL5_RFRSH_1;
                    PL_5_Time_out <= 0;
                end
                default: begin
                    RDBCK_Pl5 <= PL5_RFRSH_1;
                end
            endcase
        end
    end 


    always @(posedge i_clk_100 or negedge w_rst_n) begin
        if (~w_rst_n) begin
            r_plank6_done <= 1'b0;
            r_PLANK6_SNSR_data <= 0;
            r_plank6_cnt <= 0;
            PL_6_Time_out <= 0;
            RDBCK_Pl6 <= PL6_RFRSH_1;
        end else begin
            r_plank6_done <= 1'b0;
            case (RDBCK_Pl6)
                PL6_RFRSH_1 : begin
                    RDBCK_Pl6 <= PL6_RFRSH_1;
                    if (w_SNSR_data_valid) begin
                        RDBCK_Pl6 <= PL6_RFRSH_2;
                    end
                end
                PL6_RFRSH_2 : begin
                    PL_6_Time_out <= PL_6_Time_out + 1'b1;

                    if (PL_6_Time_out > 32'd1110000) begin
                        RDBCK_Pl6 <= PL6_SM3;
                        r_PLANK6_SNSR_data <= 0;
                    end

                    if ((r_plank_data_module[5] == 8'hCC) && r_plank_module_valid[5]) begin
                        PL_6_Time_out <= 0;
                        r_PLANK6_SNSR_data <= 0;
                        RDBCK_Pl6 <= PL6_SM2;
                    end 
                end
                PL6_SM2 : begin
                    if (r_plank_module_valid[5])begin
                        r_PLANK6_SNSR_data <= (r_PLANK6_SNSR_data << 8) | r_plank_data_module[5];
                        r_plank6_cnt <= r_plank6_cnt + 1'b1;
                    end 
                    if (r_plank6_cnt < 5'd13) begin
                        RDBCK_Pl6 <= PL6_SM2;
                    end else begin
                        RDBCK_Pl6 <= PL6_SM3;
                        r_plank6_cnt <= 5'd0;
                    end
                end 
                PL6_SM3 : begin
                    r_plank6_done <= 1'b1;
                    RDBCK_Pl6 <= PL6_RFRSH_1;
                    PL_6_Time_out <= 0;
                end
                default: begin
                    RDBCK_Pl6 <= PL6_RFRSH_1;
                end
            endcase
        end
    end 

    always @(posedge i_clk_100 or negedge w_rst_n) begin
        if (~w_rst_n) begin
            r_plank7_done <= 1'b0;
            r_PLANK7_SNSR_data <= 0;
            r_plank7_cnt <= 0;
            PL_7_Time_out <= 0;
            RDBCK_Pl7 <= PL7_RFRSH_1;
        end else begin
            r_plank7_done <= 1'b0;
            case (RDBCK_Pl7)
                PL7_RFRSH_1 : begin
                    RDBCK_Pl7 <= PL7_RFRSH_1;
                    if (w_SNSR_data_valid) begin
                        RDBCK_Pl7 <= PL7_RFRSH_2;
                    end
                end
                PL7_RFRSH_2 : begin
                    PL_7_Time_out <= PL_7_Time_out + 1'b1;

                    if (PL_7_Time_out > 32'd1110000) begin
                        RDBCK_Pl7 <= PL7_SM3;
                        r_PLANK7_SNSR_data <= 0;
                    end

                    if ((r_plank_data_module[6] == 8'hCC) && r_plank_module_valid[6]) begin
                        PL_7_Time_out <= 0;
                        r_PLANK7_SNSR_data <= 0;
                        RDBCK_Pl7 <= PL7_SM2;
                    end 
                end
                PL7_SM2 : begin
                    if (r_plank_module_valid[6])begin
                        r_PLANK7_SNSR_data <= (r_PLANK7_SNSR_data << 8) | r_plank_data_module[6];
                        r_plank7_cnt <= r_plank7_cnt + 1'b1;
                    end 
                    if (r_plank7_cnt < 5'd13) begin
                        RDBCK_Pl7 <= PL7_SM2;
                    end else begin
                        RDBCK_Pl7 <= PL7_SM3;
                        r_plank7_cnt <= 5'd0;
                    end
                end 
                PL7_SM3 : begin
                    r_plank7_done <= 1'b1;
                    RDBCK_Pl7 <= PL7_RFRSH_1;
                    PL_7_Time_out <= 0;
                end
                default: begin
                    RDBCK_Pl7 <= PL7_RFRSH_1;
                end
            endcase
        end
    end 


    always @(posedge i_clk_100 or negedge w_rst_n) begin
        if (~w_rst_n) begin
            r_plank8_done <= 1'b0;
            r_PLANK8_SNSR_data <= 0;
            r_plank8_cnt <= 0;
            PL_8_Time_out <= 0;
            RDBCK_Pl8 <= PL8_RFRSH_1;
        end else begin
            r_plank8_done <= 1'b0;
            case (RDBCK_Pl8)
                PL8_RFRSH_1 : begin
                    RDBCK_Pl8 <= PL8_RFRSH_1;
                    if (w_SNSR_data_valid) begin
                        RDBCK_Pl8 <= PL8_RFRSH_2;
                    end
                end
                PL8_RFRSH_2 : begin
                    PL_8_Time_out <= PL_8_Time_out + 1'b1;

                    if (PL_8_Time_out > 32'd1110000) begin
                        RDBCK_Pl8 <= PL8_SM3;
                        r_PLANK8_SNSR_data <= 0;
                    end

                    if ((r_plank_data_module[7] == 8'hCC) && r_plank_module_valid[7]) begin
                        PL_8_Time_out <= 0;
                        r_PLANK8_SNSR_data <= 0;
                        RDBCK_Pl8 <= PL8_SM2;
                    end 
                end
                PL8_SM2 : begin
                    if (r_plank_module_valid[7])begin
                        r_PLANK8_SNSR_data <= (r_PLANK8_SNSR_data << 8) | r_plank_data_module[7];
                        r_plank8_cnt <= r_plank8_cnt + 1'b1;
                    end 
                    if (r_plank8_cnt < 5'd13) begin
                        RDBCK_Pl8 <= PL8_SM2;
                    end else begin
                        RDBCK_Pl8 <= PL8_SM3;
                        r_plank8_cnt <= 5'd0;
                    end
                end 
                PL8_SM3 : begin
                    r_plank8_done <= 1'b1;
                    RDBCK_Pl8 <= PL8_RFRSH_1;
                    PL_8_Time_out <= 0;
                end
                default: begin
                    RDBCK_Pl8 <= PL8_RFRSH_1;
                end
            endcase
        end
    end 


    reg r_all_PLNK_done;

    always @(posedge i_clk_100 or negedge w_rst_n) begin
        if (~w_rst_n) begin
            r_all_PLNK_done <= 1'd0;
            r_PLNK_all_data <= 0;
        end else begin
            r_all_PLNK_done <= 1'd0;

            if (r_plank1_done) begin
                r_plank1_pending_done <= 1'b1;
            end

            if (r_plank2_done) begin
                r_plank2_pending_done <= 1'b1;
            end

            if (r_plank3_done) begin
                r_plank3_pending_done <= 1'b1;
            end

            if (r_plank4_done) begin
                r_plank4_pending_done <= 1'b1;
            end

            if (r_plank5_done) begin
                r_plank5_pending_done <= 1'b1;
            end

            if (r_plank6_done) begin
                r_plank6_pending_done <= 1'b1;
            end

            if (r_plank7_done) begin
                r_plank7_pending_done <= 1'b1;
            end

            if (r_plank8_done) begin
                r_plank8_pending_done <= 1'b1;
            end

            if (r_plank1_pending_done && r_plank2_pending_done && r_plank3_pending_done && r_plank4_pending_done && r_plank5_pending_done && r_plank6_pending_done && r_plank7_pending_done && r_plank8_pending_done) begin
                r_PLNK_all_data <= {8'hCC,r_PLANK1_SNSR_data,8'd1,r_PLANK2_SNSR_data,8'd2,r_PLANK3_SNSR_data,8'd3,r_PLANK4_SNSR_data,8'd4,r_PLANK5_SNSR_data,8'd5,r_PLANK6_SNSR_data,8'd6,r_PLANK7_SNSR_data,8'd7,r_PLANK8_SNSR_data,8'd8};
                r_all_PLNK_done <= 1'd1;
                r_plank1_pending_done <= 1'b0;
                r_plank2_pending_done <= 1'b0;
                r_plank3_pending_done <= 1'b0;
                r_plank4_pending_done <= 1'b0;
                r_plank5_pending_done <= 1'b0;
                r_plank6_pending_done <= 1'b0;
                r_plank7_pending_done <= 1'b0;
                r_plank8_pending_done <= 1'b0;
            end
        end
    end

    reg[15:0] r_ACK_PLNK_data;
    reg[31:0] r_buff_ACK_PLNK_data;
    reg[2:0] r_PLNCK_send;
    reg[2:0] r_PLNK_ACK_cnt;


    reg[1:0]SM_ACU_RDBCK;
    localparam ACU_SM1 = 2'd0;
    localparam ACU_SM2 = 2'd1;
    
    always @(posedge i_clk_100 or negedge w_rst_n) begin
        if (~w_rst_n) begin
            r_ACK_send <= 7'd0;
            SM_Sensor_Fdcbk <= SM_Sensor_wait;
            r_ACK_FDBCK_done <= 1'b0;
            r_uart_tx_valid_fdbck <= 1'b0;
            r_uart_tx_data_fdbck <= 7'd0;
            r_buff_ACK_FDBCK_done <= 1'b0;
            r_PLNK_buff_all_data <= 0;
            r_ACK_PLNK_data <= 15'd0;
            r_buff_ACK_PLNK_data <= 32'd0;
            r_ACK_FDBCK_done <= 1'b0;
            r_PLNCK_send <= 3'd0;
            r_PLNK_ACK_cnt <= 3'd0;
            SM_ACU_RDBCK <= ACU_SM1;
            SM_ACK_FDBCK <= SM_Fdbck_ACK_1;
        end else begin
            r_uart_tx_valid_fdbck <= 1'b0;

            case (SM_ACU_RDBCK)
                ACU_SM1: begin
                    SM_ACU_RDBCK <= ACU_SM1;
                    if (r_PLANK_data_valid_buff_2) begin
                        SM_ACU_RDBCK <= ACU_SM2;
                    end
                end
                ACU_SM2 : begin
                    SM_ACU_RDBCK <= ACU_SM2;
                    if ((w_uart_rx_data_fdbck_Plank[r_Plank_module] == 8'hEE) && w_uart_rx_valid_fdbck_Plank[r_Plank_module] )begin//w_uart_rx_data_fdbck_Plank[r_Plank_module] == 8'hEE) begin
                        r_ACK_FDBCK_done <= 1'b1;
                        SM_ACU_RDBCK <= ACU_SM1;
                    end
                end
                default: begin
                    SM_ACU_RDBCK <= ACU_SM1;
                end
            endcase

            r_buff_ACK_FDBCK_done <= r_ACK_FDBCK_done;

            if (~w_uart_tx_active_fdbck && r_ACK_FDBCK_done) begin
                case (SM_ACK_FDBCK)
                    SM_Fdbck_ACK_1: begin
                        SM_ACK_FDBCK <= SM_Fdbck_ACK_1;
                        if (w_uart_rx_valid_fdbck_Plank[r_Plank_module]) begin

                            r_ACK_PLNK_data <= (r_ACK_PLNK_data << 8) | w_uart_rx_data_fdbck_Plank[r_Plank_module];
                            r_PLNK_ACK_cnt <= r_PLNK_ACK_cnt + 1'b1;

                            if (r_PLNK_ACK_cnt < 3'd1) begin
                                SM_ACK_FDBCK <= SM_Fdbck_ACK_1;
                            end else begin
                                SM_ACK_FDBCK <= SM_Fdbck_ACK_4;
                                r_PLNK_ACK_cnt <= 3'd0;
                            end
                        end
                    end
                    SM_Fdbck_ACK_4 : begin
                        r_buff_ACK_PLNK_data <= {8'hEE,r_ACK_PLNK_data,r_buff_Plank_module};
                        SM_ACK_FDBCK <= SM_Fdbck_ACK_2;
                    end
                    SM_Fdbck_ACK_2 : begin
                        if (r_PLNCK_send < 3'd4) begin
                            r_uart_tx_data_fdbck <= r_buff_ACK_PLNK_data[31:24];
                            r_buff_ACK_PLNK_data <= (r_buff_ACK_PLNK_data << 8);
                            r_uart_tx_valid_fdbck <= 1'b1;
                            r_PLNCK_send <= r_PLNCK_send + 1'b1;
                            SM_ACK_FDBCK <= SM_Fdbck_ACK_2;
                        end else begin
                            SM_ACK_FDBCK <= SM_Fdbck_ACK_3;
                            r_PLNCK_send <= 3'd0;
                        end
                    end
                    SM_Fdbck_ACK_3 : begin
                        SM_ACK_FDBCK <= SM_Fdbck_ACK_1;
                        r_ACK_FDBCK_done <= 1'b0;
                    end
                    default: begin
                        SM_ACK_FDBCK <= SM_Fdbck_ACK_1;
                    end
                endcase
            end

            if (~w_uart_tx_active_fdbck && ~r_buff_ACK_FDBCK_done) begin
                case (SM_Sensor_Fdcbk)
                    SM_Sensor_wait : begin
                        SM_Sensor_Fdcbk <= SM_Sensor_wait;
                        if (r_all_PLNK_done) begin
                            SM_Sensor_Fdcbk <= SM_SNSR_STRT;
                            r_PLNK_buff_all_data <= r_PLNK_all_data;
                        end
                    end
                    SM_SNSR_STRT: begin
                        if (r_ACK_send < 7'd113) begin
                            r_uart_tx_data_fdbck <= r_PLNK_buff_all_data[903:896];
                            r_PLNK_buff_all_data <= (r_PLNK_buff_all_data << 8);
                            r_uart_tx_valid_fdbck <= 1'b1;
                            r_ACK_send <= r_ACK_send + 1'b1;
                            SM_Sensor_Fdcbk <= SM_SNSR_STRT;
                        end else begin
                            SM_Sensor_Fdcbk <= SM_Sensor_wait;
                            r_ACK_send <= 7'd0;
                        end
                    end
                    default: begin
                        SM_Sensor_Fdcbk <= SM_Sensor_wait;
                    end
                endcase
            end
        end
    end 

    always @(posedge i_clk_100 or negedge w_rst_n) begin
        if (~w_rst_n) begin
            r_uart_tx_data      <= 0;
            r_uart_tx_valid     <= 0;
            r_send_counter      <= 0;
            r_send_plank     <= 0;
            send_plank_data     <= SM_1;
        end else begin
            r_uart_tx_valid <= 0;
            if (~w_uart_tx_active) begin
                case (send_plank_data)
                    SM_1:begin  
                        send_plank_data <= SM_1;
                        if (r_PLANK_data_valid_buff_3) begin
                            send_plank_data <= SM_2;
                            r_send_plank    <= r_plank_data;
                        end
                    end 
                    SM_2 : begin
                        if(r_send_counter < 5'd19) begin
                            r_uart_tx_data <= r_send_plank[7:0];
                            r_uart_tx_valid <= 1;
                            r_send_plank <= (r_send_plank >> 8);
                            r_send_counter <= r_send_counter + 1;
                        end else begin
                            r_send_counter <= 0;
                            send_plank_data <= SM_3;
                        end
                    end
                    SM_3 : begin
                        r_uart_tx_data <= w_checksum;
                        r_uart_tx_valid <= 1;
                        send_plank_data <= SM_4;
                    end
                    SM_4 : begin
                        r_uart_tx_data <= 8'h55;
                        r_uart_tx_valid <= 1;
                        send_plank_data <= SM_1;
                    end
                    default: begin
                        send_plank_data <= SM_1;
                    end
                endcase
            end
        end
    end

    genvar j;
    generate 
        for (j = 0;j < 4;j = j + 1) begin : gen_assign_var
            assign o_enable[j] = r_enable[j];
        end
    endgenerate

    genvar k;
    generate 
        for (k = 0;k < 8;k = k + 1) begin : gen_assign_fdbck
            assign w_rx_serial_fdbck_Plank[k] = i_fdbck_plank[k];
        end
    endgenerate

    assign o_ATT1       = r_ATT1;
    assign o_ATT2       = r_ATT2;
    assign o_ATT3       = r_ATT3;
    assign o_ATT4       = r_ATT4;

    assign o_tx_serial[0] = (r_Plank_module == 3'd0) ? w_tx_serial : 1'b1; 
    assign o_tx_serial[1] = (r_Plank_module == 3'd1) ? w_tx_serial : 1'b1; 
    assign o_tx_serial[2] = (r_Plank_module == 3'd2) ? w_tx_serial : 1'b1; 
    assign o_tx_serial[3] = (r_Plank_module == 3'd3) ? w_tx_serial : 1'b1; 
    assign o_tx_serial[4] = (r_Plank_module == 3'd4) ? w_tx_serial : 1'b1; 
    assign o_tx_serial[5] = (r_Plank_module == 3'd5) ? w_tx_serial : 1'b1; 
    assign o_tx_serial[6] = (r_Plank_module == 3'd6) ? w_tx_serial : 1'b1; 
    assign o_tx_serial[7] = (r_Plank_module == 3'd7) ? w_tx_serial : 1'b1; 
    
    //w_rst_n 
    
    assign w_rest_control0 = (r_Plank_module == 3'd0) ? ~w_rst_PLANK : 1'b1;
    assign w_rest_control1 = (r_Plank_module == 3'd1) ? ~w_rst_PLANK : 1'b1;
    assign w_rest_control2 = (r_Plank_module == 3'd2) ? ~w_rst_PLANK : 1'b1;
    assign w_rest_control3 = (r_Plank_module == 3'd3) ? ~w_rst_PLANK : 1'b1;
    assign w_rest_control4 = (r_Plank_module == 3'd4) ? ~w_rst_PLANK : 1'b1;
    assign w_rest_control5 = (r_Plank_module == 3'd5) ? ~w_rst_PLANK : 1'b1;
    assign w_rest_control6 = (r_Plank_module == 3'd6) ? ~w_rst_PLANK : 1'b1;
    assign w_rest_control7 = (r_Plank_module == 3'd7) ? ~w_rst_PLANK : 1'b1;
    
    
    assign o_reset_plank[0] = (w_rst_n == 1'b0 || w_rest_control0 == 1'b0) ? 1'b0 : 1'b1 ; 
    assign o_reset_plank[1] = (w_rst_n == 1'b0 || w_rest_control1 == 1'b0) ? 1'b0 : 1'b1 ;  
    assign o_reset_plank[2] = (w_rst_n == 1'b0 || w_rest_control2 == 1'b0) ? 1'b0 : 1'b1 ;  
    assign o_reset_plank[3] = (w_rst_n == 1'b0 || w_rest_control3 == 1'b0) ? 1'b0 : 1'b1 ;  
    assign o_reset_plank[4] = (w_rst_n == 1'b0 || w_rest_control4 == 1'b0) ? 1'b0 : 1'b1 ;  
    assign o_reset_plank[5] = (w_rst_n == 1'b0 || w_rest_control5 == 1'b0) ? 1'b0 : 1'b1 ;  
    assign o_reset_plank[6] = (w_rst_n == 1'b0 || w_rest_control6 == 1'b0) ? 1'b0 : 1'b1 ;  
    assign o_reset_plank[7] = (w_rst_n == 1'b0 || w_rest_control7 == 1'b0) ? 1'b0 : 1'b1 ;  

    assign o_BITE_CNTRL = r_BITE_CNTRL;
    assign o_SUB_ARRAY  = r_SUB_ARRAY;

    assign o_attn_31p5_0 = ((r_Plank_module == 3'd0) && (r_soft_inhibit && i_inhibit)) ? i_attn_31p5 : 1'b0;
    assign o_attn_31p5_1 = ((r_Plank_module == 3'd1) && (r_soft_inhibit && i_inhibit)) ? i_attn_31p5 : 1'b0;
    assign o_attn_31p5_2 = ((r_Plank_module == 3'd2) && (r_soft_inhibit && i_inhibit)) ? i_attn_31p5 : 1'b0;
    assign o_attn_31p5_3 = ((r_Plank_module == 3'd3) && (r_soft_inhibit && i_inhibit)) ? i_attn_31p5 : 1'b0;
    assign o_attn_31p5_4 = ((r_Plank_module == 3'd4) && (r_soft_inhibit && i_inhibit)) ? i_attn_31p5 : 1'b0;
    assign o_attn_31p5_5 = ((r_Plank_module == 3'd5) && (r_soft_inhibit && i_inhibit)) ? i_attn_31p5 : 1'b0;
    assign o_attn_31p5_6 = ((r_Plank_module == 3'd6) && (r_soft_inhibit && i_inhibit)) ? i_attn_31p5 : 1'b0;
    assign o_attn_31p5_7 = ((r_Plank_module == 3'd7) && (r_soft_inhibit && i_inhibit)) ? i_attn_31p5 : 1'b0;

    assign o_phase_180_0 = ((r_Plank_module == 3'd0) && (r_soft_inhibit && i_inhibit)) ? i_phase_180 : 1'b0;
    assign o_phase_180_1 = ((r_Plank_module == 3'd1) && (r_soft_inhibit && i_inhibit)) ? i_phase_180 : 1'b0;
    assign o_phase_180_2 = ((r_Plank_module == 3'd2) && (r_soft_inhibit && i_inhibit)) ? i_phase_180 : 1'b0;
    assign o_phase_180_3 = ((r_Plank_module == 3'd3) && (r_soft_inhibit && i_inhibit)) ? i_phase_180 : 1'b0;
    assign o_phase_180_4 = ((r_Plank_module == 3'd4) && (r_soft_inhibit && i_inhibit)) ? i_phase_180 : 1'b0;
    assign o_phase_180_5 = ((r_Plank_module == 3'd5) && (r_soft_inhibit && i_inhibit)) ? i_phase_180 : 1'b0;
    assign o_phase_180_6 = ((r_Plank_module == 3'd6) && (r_soft_inhibit && i_inhibit)) ? i_phase_180 : 1'b0;
    assign o_phase_180_7 = ((r_Plank_module == 3'd7) && (r_soft_inhibit && i_inhibit)) ? i_phase_180 : 1'b0;

    assign o_TR_pulse_0 = ((r_Plank_module == 3'd0) && (r_soft_inhibit && i_inhibit)) ? i_TR_pulse : 1'b0;
    assign o_TR_pulse_1 = ((r_Plank_module == 3'd1) && (r_soft_inhibit && i_inhibit)) ? i_TR_pulse : 1'b0;
    assign o_TR_pulse_2 = ((r_Plank_module == 3'd2) && (r_soft_inhibit && i_inhibit)) ? i_TR_pulse : 1'b0;
    assign o_TR_pulse_3 = ((r_Plank_module == 3'd3) && (r_soft_inhibit && i_inhibit)) ? i_TR_pulse : 1'b0;
    assign o_TR_pulse_4 = ((r_Plank_module == 3'd4) && (r_soft_inhibit && i_inhibit)) ? i_TR_pulse : 1'b0;
    assign o_TR_pulse_5 = ((r_Plank_module == 3'd5) && (r_soft_inhibit && i_inhibit)) ? i_TR_pulse : 1'b0;
    assign o_TR_pulse_6 = ((r_Plank_module == 3'd6) && (r_soft_inhibit && i_inhibit)) ? i_TR_pulse : 1'b0;
    assign o_TR_pulse_7 = ((r_Plank_module == 3'd7) && (r_soft_inhibit && i_inhibit)) ? i_TR_pulse : 1'b0;

    assign o_inhibit_0 = ((r_Plank_module == 3'd0) && (r_soft_inhibit && i_inhibit)) ? i_inhibit : 1'b0;
    assign o_inhibit_1 = ((r_Plank_module == 3'd1) && (r_soft_inhibit && i_inhibit)) ? i_inhibit : 1'b0;
    assign o_inhibit_2 = ((r_Plank_module == 3'd2) && (r_soft_inhibit && i_inhibit)) ? i_inhibit : 1'b0;
    assign o_inhibit_3 = ((r_Plank_module == 3'd3) && (r_soft_inhibit && i_inhibit)) ? i_inhibit : 1'b0;
    assign o_inhibit_4 = ((r_Plank_module == 3'd4) && (r_soft_inhibit && i_inhibit)) ? i_inhibit : 1'b0;
    assign o_inhibit_5 = ((r_Plank_module == 3'd5) && (r_soft_inhibit && i_inhibit)) ? i_inhibit : 1'b0;
    assign o_inhibit_6 = ((r_Plank_module == 3'd6) && (r_soft_inhibit && i_inhibit)) ? i_inhibit : 1'b0;
    assign o_inhibit_7 = ((r_Plank_module == 3'd7) && (r_soft_inhibit && i_inhibit)) ? i_inhibit : 1'b0;

endmodule
