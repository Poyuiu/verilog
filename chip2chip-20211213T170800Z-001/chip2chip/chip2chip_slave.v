`timescale 1ns / 1ps
module slave_control(
        clk,
        rst_n,
        request, ack,
        data_in, notice,
        valid,
        data
    );
    input clk;
    input rst_n;
    input request;
    input [3-1:0] data_in;
    input valid;
    output reg ack;
    output reg notice;
    output reg [3-1:0] data;

    parameter state_wait_rqst = 2'b00;
    parameter state_wait_to_send_ack = 2'b01;
    parameter state_send_ack = 2'b10;
    parameter state_wait_data = 2'b11;

    reg [2-1:0] state, next_state;
    reg start, next_start;
    reg next_ack;
    reg next_notice;
    reg [3-1:0] next_data;
    wire done;
    counter cnt_0(
                .clk(clk),
                .rst_n(rst_n),
                .start(start),
                .done(done)
            );

    always@(posedge clk) begin
        if (rst_n == 0) begin
            state = state_wait_rqst;
            notice = 0;
            ack = 0;
            data = 0;
            start = 0;
        end
        else begin
            state <= next_state;
            notice <= next_notice;
            ack <= next_ack;
            data <= next_data;
            start <= next_start;
        end
    end

    always@(*) begin
        next_state = state;
        next_notice = notice;
        next_ack = ack;
        next_data = data;
        next_start = start;
        case(state)
            state_wait_rqst: begin
                next_state = (request == 1) ?
                state_wait_to_send_ack : state_wait_rqst;
                next_notice = 1'b0;
                next_ack = 1'b0;
                next_data = 3'b000;
                next_start = (request == 1) ? 1'b1:1'b0;
            end
            state_wait_to_send_ack: begin
                next_state = (done == 1) ?
                state_wait_data : state_wait_to_send_ack;
                next_notice = (done == 1) ? 1'b0 : 1'b1;
                next_ack = (done == 1) ? 1'b1 : 1'b0;
                next_data = 3'b000;
                next_start = (done == 1) ? 1'b0 : 1'b1;
            end
            state_wait_data: begin
                next_state = (valid == 1) ?
                state_wait_data : state_wait_rqst;
                next_notice = 1'b0;
                next_ack = 1'b1;
                next_data = (valid == 1) ? data_in : 3'b000;
                next_start = 1'b0;
            end
            default: begin
            end
        endcase
    end
endmodule

module counter(
        clk,
        rst_n,
        start,
        done
    );
    input clk;
    input rst_n;
    input start;
    output reg done;

    reg [27-1:0] count, next_count;

    always@(posedge clk) begin
        if (rst_n == 0) begin
            count = 0;
        end
        else begin
            count <= next_count;
        end
    end

    always@(*) begin
        next_count = count;
        if (start) begin
            if (count == 27'd100000000) begin
                done = 1;
                next_count = 0;
            end
            else begin
                next_count = count + 1;
                done = 0;
            end
        end
        else begin
            done = 0;
            next_count = 0;
        end
    end
endmodule

module decoder(in, out);
    input [3-1:0] in;
    output reg [7:0] out;
    always@(*) begin
        case(in)
            3'b000:
                out = 8'b0000_0001;
            3'b001:
                out = 8'b0000_0010;
            3'b010:
                out = 8'b0000_0100;
            3'b011:
                out = 8'b0000_1000;
            3'b100:
                out = 8'b0001_0000;
            3'b101:
                out = 8'b0010_0000;
            3'b110:
                out = 8'b0100_0000;
            3'b111:
                out = 8'b1000_0000;
        endcase
    end
endmodule

module seven_segment(in, out);
    input [8-1:0] in;
    output reg [7-1:0] out;
    always@(*) begin
        out[0] = (in[1]|in[4]);
        out[1] = (in[5]|in[6]);
        out[2] = (in[2]);
        out[3] = (in[1]|in[4]|in[7]);
        out[4] = (in[1]|in[3]|in[4]|in[5]|in[7]);
        out[5] = (in[1]|in[2]|in[3]|in[7]);
        out[6] = (in[0]|in[1]|in[7]);
    end
endmodule

module debounce (pb_debounced, pb, clk);
    output pb_debounced;
    input pb;
    input clk;

    reg [3:0] DFF;
    always @(posedge clk) begin
        DFF[3:1] <= DFF[2:0];
        DFF[0] <= pb;
    end

    assign pb_debounced = ((DFF == 4'b1111) ? 1'b1 : 1'b0);

endmodule

module onepulse (pb_debounced, clock, pb_one_pulse);
    input pb_debounced;
    input clock;
    output reg pb_one_pulse;
    reg pb_debounced_delay;
    always @(posedge clock) begin
        pb_one_pulse <= pb_debounced & (! pb_debounced_delay);
        pb_debounced_delay <= pb_debounced;
    end
endmodule

module top(
        input clk,
        input rst_n,
        input request,
        input valid,
        output [7-1:0] seven_seg,
        output notice_slave,
        output [4-1:0] AN,
        input [3-1:0] data_in,
        output ack
    );

    wire rst_n_inv;
    wire [3-1:0]slave_data_o;
    wire [8-1:0]slave_data_dec;
    wire db_rst_n, op_rst_n;

    assign rst_n_inv = ~op_rst_n;
    assign AN = 4'b1110;

    debounce db_0(
                 .pb_debounced(db_rst_n),
                 .pb(rst_n),
                 .clk(clk)
             );
    onepulse op_0(
                 .pb_debounced(db_rst_n),
                 .clock(clk),
                 .pb_one_pulse(op_rst_n)
             );
    slave_control sl_ctrl_0(
                      .clk(clk),
                      .rst_n(rst_n_inv),
                      .request(request),
                      .ack(ack),
                      .data_in(data_in),
                      .notice(notice_slave),
                      .valid(valid),
                      .data(slave_data_o)
                  );
    decoder dec0(
                .in(slave_data_o),
                .out(slave_data_dec)
            );
    seven_segment dis_0(
                      .in(slave_data_dec),
                      .out(seven_seg)
                  );
endmodule
