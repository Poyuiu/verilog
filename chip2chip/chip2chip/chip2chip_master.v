`timescale 1ns / 1ps
module master_control(
        clk,
        rst_n,
        request,
        ack,
        data_in,
        notice,
        data,
        valid,
        request2s
    );
    input clk;
    input rst_n;
    input request;
    input ack;
    input [3-1: 0] data_in;
    output reg request2s;
    output reg notice;
    output reg [3-1:0] data;
    output reg valid;

    parameter state_wait_rqst = 3'b000;
    parameter state_wait_ack  = 3'b001;
    parameter state_wait_to_send_data = 3'b100;

    parameter state_send_data = 3'b101;

    reg [3-1:0] state, next_state;
    reg next_notice;
    reg [3-1:0] next_data;
    reg next_request2s;
    reg start, next_start;
    reg next_valid;

    wire done;

    counter cnt_0(
                .clk(clk),
                .rst_n(rst_n),
                .start(start),
                .done(done)
            );

    always@(posedge clk) begin
        if (rst_n == 0) begin
            notice = 1'b0;
            state = state_wait_rqst;
            data = 3'b0;
            request2s = 1'b0;
            start = 1'b0;
            valid = 1'b0;
        end
        else begin
            notice <= next_notice;
            state <= next_state;
            data <= next_data;
            request2s <= next_request2s;
            start <= next_start;
            valid <= next_valid;
        end
    end

    always@(*) begin
        next_state = state;
        next_notice = notice;
        next_data = data;
        next_request2s = request2s;
        next_start = start;
        next_valid = valid;
        case(state)
            state_wait_rqst: begin
                next_state = (request == 1'b1) ?
                state_wait_ack: state_wait_rqst;
                next_notice = 1'b0;
                next_data = 3'b000;
                next_request2s = (request == 1'b1) ? 1'b1: 1'b0;
                next_start = 1'b0;
                next_valid = 1'b0;
            end
            state_wait_ack: begin
                next_state = (ack == 1'b1) ?
                state_wait_to_send_data: state_wait_ack;
                next_notice = 1'b0;
                next_data = 3'b000;
                next_request2s = (ack == 1'b1) ? 1'b0 : 1'b1;
                next_start = (ack == 1'b1)? 1'b1: 1'b0;
                next_valid = 1'b0;
            end
            state_wait_to_send_data: begin
                next_state = (done == 1'b1) ?
                state_send_data: state_wait_to_send_data;
                next_notice = (done == 1'b1) ? 1'b0 : 1'b1;
                next_data = (done == 1'b1) ? data_in: 3'b000;
                next_request2s = 1'b0;
                next_start = (done == 1'b1) ? 1'b0 : 1'b1;
                next_valid = (done == 1) ? 1'b1: 1'b0;
            end
            state_send_data: begin
                next_state = (ack == 1'b0) ?
                state_wait_rqst: state_send_data;
                next_notice = 1'b0;
                next_data = (ack == 1'b0) ? 3'b000: data_in;
                next_request2s = 1'b0;
                next_start = 1'b0;
                next_valid = (ack == 1'b0) ? 1'b0: 1'b1;
            end
            default: begin
            end
        endcase
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
        clk,
        rst_n,
        in,
        request,
        notice_master,
        data_to_slave_o,
        valid, request2s,
        ack, seven_seg,
        AN
    );
    input clk;
    input rst_n;
    input [8-1:0] in;
    input request;
    input ack;
    output [3-1:0] data_to_slave_o;
    output notice_master;
    output valid;
    output request2s;
    output [7-1:0] seven_seg;
    output [4-1:0] AN;
    wire db_request;
    wire op_request;
    wire [3-1:0] data_to_slave;
    wire rst_n_inv;
    wire [8-1:0]slave_data_dec;
    wire db_rst_n, op_rst_n;
    wire [8-1:0]data_to_slave_dec;
    assign rst_n_inv = ~op_rst_n;
    assign AN = 4'b1110;
    assign data_to_slave_o = data_to_slave;
    encoder enc0(.in(in), .out(data_to_slave));
    debounce db_0(
                 .pb_debounced(db_request),
                 .pb(request),
                 .clk(clk)
             );
    onepulse op_0(
                 .pb_debounced(db_request),
                 .clock(clk),
                 .pb_one_pulse(op_request)
             );
    debounce db_1(.pb_debounced(db_rst_n),
                  .pb(rst_n),
                  .clk(clk)
                 );
    onepulse op_1(
                 .pb_debounced(db_rst_n),
                 .clock(clk),
                 .pb_one_pulse(op_rst_n)
             );
    master_control ms_ctrl_0(
                       .clk(clk),
                       .rst_n(rst_n_inv),
                       .request(op_request),
                       .ack(ack),
                       .data_in(data_to_slave),
                       .notice(notice_master),
                       .data(),
                       .valid(valid),
                       .request2s(request2s)
                   );
    decoder dec0(
                .in(data_to_slave),
                .out(data_to_slave_dec)
            );
    seven_segment dis_0(
                      .in(data_to_slave_dec),
                      .out(seven_seg)
                  );
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

module encoder(in, out);
    input [8-1:0] in;
    output reg [3-1:0] out;
    always@(*) begin
        case(in)
            8'b0000_0001:
                out = 3'd0;
            8'b0000_0010:
                out = 3'd1;
            8'b0000_0100:
                out = 3'd2;
            8'b0000_1000:
                out = 3'd3;
            8'b0001_0000:
                out = 3'd4;
            8'b0010_0000:
                out = 3'd5;
            8'b0100_0000:
                out = 3'd6;
            8'b1000_0000:
                out = 3'd7;
            default:
                out = 0;
        endcase
    end
endmodule

module counter(clk, rst_n, start, done);
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
