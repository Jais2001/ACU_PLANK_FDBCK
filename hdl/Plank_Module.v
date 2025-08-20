module PLANK #(
    parameter temp = 25'h02020DD,
    parameter ADC_data = 96'hACDEADBEEFDEADBEEFABAACC
)(
    input wire i_clk,
    input wire i_rst,
    input wire i_rx_serial,
    
    output wire o_tx_serial

);

    // Set Parameter CLKS_PER_BIT as follows:
    // CLKS_PER_BIT = (Frequency of i_Clock)/(Frequency of UART)
    // Example: 100 MHz Clock, 115200 baud UART
    // (100000000)/(115200) = 868

    localparam CLOCK_FREQ = 100000000;
    localparam BAUD_RATE = 115200;
    localparam CLKS_PER_BIT = CLOCK_FREQ/BAUD_RATE;

    reg [7:0] r_ch_power;
    reg [5:0] r_attn_ch1;
    reg [5:0] r_attn_ch2;
    reg [5:0] r_attn_ch3;
    reg [5:0] r_attn_ch4;
    reg [5:0] r_attn_ch5;
    reg [5:0] r_attn_ch6;
    reg [5:0] r_attn_ch7;
    reg [5:0] r_attn_ch8;
    reg [5:0] r_phase_ch1;
    reg [5:0] r_phase_ch2;
    reg [5:0] r_phase_ch3;
    reg [5:0] r_phase_ch4;
    reg [5:0] r_phase_ch5;
    reg [5:0] r_phase_ch6;
    reg [5:0] r_phase_ch7;
    reg [5:0] r_phase_ch8;
    reg       r_tx_rx_sel;
    reg [1:0] r_band_sel;

    reg r_soft_inhibit;

    wire w_rst_n;

    wire [7:0] w_uart_rx_data;
    wire w_uart_rx_error;
    wire w_uart_rx_valid;

    reg[7:0] r_uart_rx_data;
    reg r_uart_rx_valid;

    wire w_uart_tx_active;
    reg r_uart_tx_valid = 0;
    reg [7:0] r_uart_tx_data = 0;
    wire w_uart_tx_done;
    wire w_tx_serial;

    wire[15:0] w_temp_data_GND;
    wire[15:0] w_temp_data_VCC;
    wire w_temp_ready_GND;
    wire w_temp_ready_VCC;

    reg [23:0] r_VCC_GND_Temp;
    reg[24:0]r_buff_send_Uart;
    reg r_VCC_GND_Temp_valid;
    reg [23:0] r_buff_Temp_Stats;

    reg[24:0] r_FDB_Chnnl;
    reg r_FDBCK_pending;

    reg r_ADC_valid;
    reg r_ADC_buff_Valid;
    reg r_ADC_pending;
    reg r_temp_pending;
    reg[79:0] r_ADC_data;
    reg[87:0] r_ADC_set_data;
    reg[95:0] r_buff_send_ADC;

    reg r_request_Readback;

    reg[2:0] PLANK_SM;
    localparam PLANK_Initial = 3'd1;
    localparam PLANK_Fdbck_Temp_send   = 3'd2;
    localparam PLANK_Temp_Send = 3'd3;
    localparam PLANK_ADC_send = 3'd4;

    assign w_rst_n = i_rst;


    uart_rx_PLNK #(.CLKS_PER_BIT(CLKS_PER_BIT)) uart_rx_inst
    (
        .i_Clock (i_clk),
        .i_Rx_Serial (i_rx_serial),
        .o_Rx_DV (w_uart_rx_valid),
        .o_Rx_Byte (w_uart_rx_data)
    );

    uart_tx_PLNK #(.CLKS_PER_BIT(CLKS_PER_BIT)) uart_tx_inst 
    (
        .i_Clock (i_clk),
        .i_Tx_DV (r_uart_tx_valid),
        .i_Tx_Byte (r_uart_tx_data),
        .o_Tx_Active (w_uart_tx_active),
        .o_Tx_Serial (o_tx_serial),
        .o_Tx_Done (w_uart_tx_done)
    );

    always @(posedge i_clk or negedge w_rst_n) begin
        if (w_rst_n == 1'b0) begin
            r_uart_rx_data <= 0;
            r_uart_rx_valid <= 0;
        end else begin
            r_uart_rx_data <= w_uart_rx_data;
            r_uart_rx_valid <= w_uart_rx_valid;
        end
    end

    reg [143:0] r_PLANK_data;
    reg [143:0] r_send_plank_data;

    // Variables for the PLANK packet identifier
    localparam PLANK_LENGTH = 19; // Total packet size In bytes inluding Identifier,data,checksum
    wire [(PLANK_LENGTH*8) - 1 : 0] w_PLANK_data;
    wire w_PLANK_data_valid;

    // PLANK packet identifier
    UART_packet_identifier_PLNK #(
        .RX_PACKET_LEN(PLANK_LENGTH),
        .IDENTIFIER(4'h2)
    ) read_PLANK_id_inst(
        .i_clk(i_clk),
        .i_rst_n(w_rst_n),
        .i_en(1'b1),
        .i_uart_rx_data(r_uart_rx_data),
        .o_uart_rx_error(),
        .i_uart_rx_valid(r_uart_rx_valid),
        .o_data(w_PLANK_data),
        .o_data_valid(w_PLANK_data_valid)
    );

    reg r_PLANK_data_valid_buff1;
    reg r_PLANK_data_valid_buff2;
    reg r_PLANK_data_valid_buff3;

    wire w_PLANK_FDBCK_data_valid;

    localparam send_plank_bytes = 5'd18;
    reg[17:0] r_send_counter;
    reg[1:0] r_pulse_Temp;
    reg[2:0] r_pulse_ADC;

    always @(posedge i_clk or negedge w_rst_n) begin
        if (~w_rst_n) begin
            r_VCC_GND_Temp_valid <= 1'b0;
            r_pulse_Temp <= 2'd0;
        end
        else begin
            if (r_pulse_Temp < 2'd2) begin
                r_VCC_GND_Temp_valid <= 1'b0;
                r_pulse_Temp <= r_pulse_Temp + 1;
            end else begin
                r_VCC_GND_Temp_valid <= 1'b1;
                r_pulse_Temp <= 2'd0;
            end
        end
    end

    always @(posedge i_clk or negedge w_rst_n) begin
        if (~w_rst_n) begin
            r_ADC_buff_Valid <= 1'b0;
            r_pulse_ADC <= 3'd0;
        end
        else begin
            if (r_pulse_ADC < 3'd7) begin
                r_ADC_buff_Valid <= 1'b0;
                r_pulse_ADC <= r_pulse_ADC + 1;
            end else begin
                r_ADC_buff_Valid <= 1'b1;
                r_pulse_ADC <= 2'd0;
            end
        end
    end

    always @(posedge i_clk or negedge w_rst_n) begin
        if (~w_rst_n) begin
            r_PLANK_data_valid_buff1 <= 0;
            r_PLANK_data_valid_buff2 <= 0;
            r_PLANK_data_valid_buff3 <= 0;
        end else begin
            r_PLANK_data_valid_buff1  <= w_PLANK_data_valid;
            r_PLANK_data_valid_buff2  <= r_PLANK_data_valid_buff1;
            r_PLANK_data_valid_buff3  <= r_PLANK_data_valid_buff2;
        end
    end

    always @(posedge i_clk or negedge w_rst_n) begin
        if (~w_rst_n) begin
            r_FDB_Chnnl <= 25'b0;
        end else begin
            if (r_PLANK_data_valid_buff2) begin
                r_FDB_Chnnl <= {r_tx_rx_sel,r_ch_power,8'hBB,8'hEE};
            end
        end
    end

    always @(posedge i_clk or negedge w_rst_n) begin
        if (~w_rst_n) begin
            r_ch_power  <= 0;
            r_attn_ch1  <= 0;
            r_attn_ch2  <= 0;
            r_attn_ch3  <= 0;
            r_attn_ch4  <= 0;
            r_attn_ch5  <= 0;
            r_attn_ch6  <= 0;
            r_attn_ch7  <= 0;
            r_attn_ch8  <= 0;
            r_phase_ch1 <= 0;
            r_phase_ch2 <= 0;
            r_phase_ch3 <= 0;
            r_phase_ch4 <= 0;
            r_phase_ch5 <= 0;
            r_phase_ch6 <= 0;
            r_phase_ch7 <= 0;
            r_phase_ch8 <= 0;
            r_tx_rx_sel <= 0;
            r_band_sel  <= 0;
            r_send_plank_data <= 0;
        end else begin
            if (w_PLANK_data_valid) begin
                r_send_plank_data   <= w_PLANK_data[143:0];
                r_tx_rx_sel         <=  w_PLANK_data[2];
                r_soft_inhibit      <=  w_PLANK_data[3];
                r_phase_ch1         <=  w_PLANK_data[13:8];
                r_attn_ch1          <=  w_PLANK_data[21:16];
                r_phase_ch2         <=  w_PLANK_data[29:24];
                r_attn_ch2          <=  w_PLANK_data[37:32];
                r_phase_ch3         <=  w_PLANK_data[45:40];
                r_attn_ch3          <=  w_PLANK_data[53:48];
                r_phase_ch4         <=  w_PLANK_data[61:56];
                r_attn_ch4          <=  w_PLANK_data[69:64];
                r_phase_ch5         <=  w_PLANK_data[77:72];
                r_attn_ch5          <=  w_PLANK_data[85:80];
                r_phase_ch6         <=  w_PLANK_data[93:88];
                r_attn_ch6          <=  w_PLANK_data[101:96];
                r_phase_ch7         <=  w_PLANK_data[109:104];
                r_attn_ch7          <=  w_PLANK_data[117:112];
                r_phase_ch8         <=  w_PLANK_data[125:120];
                r_attn_ch8          <=  w_PLANK_data[133:128];
                r_ch_power          <=  w_PLANK_data[143:136];
            end
        end
    end

    always @(posedge i_clk or negedge w_rst_n) begin
        if (~w_rst_n) begin
            r_send_counter <= 4'd0;
            r_uart_tx_data      <= 0;
            r_uart_tx_valid     <= 0;
            r_buff_send_Uart  <= 0;
            r_buff_send_ADC <= 0;
            PLANK_SM <= PLANK_Initial;
            r_FDBCK_pending <= 0;
            r_temp_pending <= 0;
            r_ADC_pending <= 1'b0;
        end else begin
            r_uart_tx_valid <= 0;
            if (r_PLANK_data_valid_buff3) begin
                r_FDBCK_pending <= 1'b1;
            end
            if (r_ADC_buff_Valid) begin
                r_ADC_pending <= 1'b1;
            end
            if (~w_uart_tx_active) begin
                case (PLANK_SM)
                    PLANK_Initial: begin
                        PLANK_SM <= PLANK_Initial;
                        if (r_FDBCK_pending) begin
                            PLANK_SM <= PLANK_Fdbck_Temp_send;
                            r_FDBCK_pending <= 1'b0;
                            r_buff_send_Uart <= r_FDB_Chnnl;
                        end else if (r_ADC_pending) begin
                            r_buff_send_ADC <= ADC_data;
                            r_ADC_pending <= 1'b0;
                            PLANK_SM <= PLANK_ADC_send;
                        end 
                    end
                    PLANK_Fdbck_Temp_send: begin
                        if (r_send_counter < 3'd4) begin
                            r_uart_tx_valid <= 1;
                            r_uart_tx_data  <= r_buff_send_Uart[7:0];
                            r_buff_send_Uart <= (r_buff_send_Uart >> 8);
                            r_send_counter <= r_send_counter + 1;
                        end else begin
                            r_send_counter <= 0;
                            PLANK_SM <= PLANK_Initial;
                        end
                    end
                    PLANK_ADC_send : begin
                        if (r_send_counter < 4'd12) begin
                            r_uart_tx_valid <= 1;
                            r_uart_tx_data  <= r_buff_send_ADC[7:0];
                            r_buff_send_ADC <= (r_buff_send_ADC >> 8);
                            r_send_counter <= r_send_counter + 1;
                        end else begin
                            r_send_counter <= 0;
                            PLANK_SM <= PLANK_Temp_Send;
                        end
                    end
                    PLANK_Temp_Send : begin
                        PLANK_SM <= PLANK_Temp_Send;
                        if (r_VCC_GND_Temp_valid) begin
                            r_buff_send_Uart <= temp;
                            r_temp_pending <= 1'b0;
                            PLANK_SM <= PLANK_Fdbck_Temp_send;
                        end
                    end
                    default: begin
                        PLANK_SM <= PLANK_Initial;
                    end
                endcase
            end
        end
    end

    // reg[2:0] PLANK_SM;
    // localparam PLANK_Initial = 3'd1;
    // localparam PLANK_UART_send   = 3'd2;
    // localparam PLANK_Feedbck_send = 3'd3;

    // always @(posedge i_clk or negedge w_rst_n) begin
    //     if (~w_rst_n) begin
    //         r_send_counter <= 5'b0;
    //         r_uart_tx_data      <= 0;
    //         r_uart_tx_valid     <= 0;
    //         r_buff_Temp_Stats  <= 0;
    //         PLANK_SM <= PLANK_Initial;
    //         r_FDBCK_pending <= 0;
    //     end else begin
    //         r_uart_tx_valid <= 0;
    //         if (r_PLANK_data_valid_buff3) begin
    //             r_FDBCK_pending <= 1'b1;
    //         end
    //         if (~w_uart_tx_active) begin
    //             case (PLANK_SM)
    //                 PLANK_Initial: begin
    //                     PLANK_SM <= PLANK_Initial;
    //                     if (r_FDBCK_pending) begin
    //                         PLANK_SM <= PLANK_UART_send;
    //                         r_FDBCK_pending <= 1'b0;
    //                         r_buff_Temp_Stats <= r_FDB_Chnnl;
    //                     end else if (r_VCC_GND_Temp_valid) begin
    //                         r_buff_Temp_Stats <= temp;
    //                         PLANK_SM <= PLANK_UART_send;
    //                     end
    //                 end
    //                 PLANK_UART_send: begin
    //                     if (r_send_counter < 5'd3) begin
    //                         r_uart_tx_valid <= 1;
    //                         r_uart_tx_data  <= r_buff_Temp_Stats[7:0];
    //                         r_buff_Temp_Stats <= (r_buff_Temp_Stats >> 8);
    //                         r_send_counter <= r_send_counter + 1;
    //                     end else begin
    //                         r_send_counter <= 0;
    //                         PLANK_SM <= PLANK_Initial;
    //                     end
    //                 end
    //                 default: begin
    //                     PLANK_SM <= PLANK_Initial;
    //                 end
    //             endcase
    //         end
    //     end
    // end


endmodule