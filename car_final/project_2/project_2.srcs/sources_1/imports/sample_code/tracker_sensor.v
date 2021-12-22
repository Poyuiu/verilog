`timescale 1ns/1ps
module tracker_sensor(clk, reset, left_signal, right_signal, mid_signal, state);
    input clk;
    input reset;
    input left_signal, right_signal, mid_signal;
    output reg [1:0] state;

    reg  [1:0] next_state;
    
    parameter Forward = 2'b00;
    parameter Back = 2'b01;
    parameter Left = 2'b10; // turn left "in place"
    parameter Right = 2'b11; // trun right "in place"
    
    // [TO-DO] Receive three signals and make your own policy.
    // Hint: You can use output state to change your action.
    
    always @(posedge clk) begin
        if (reset)
            state <= Forward;
        else
            state <= next_state;
    end
    
    always @* begin
        case ({left_signal, mid_signal, right_signal})
        3'b111: next_state = Forward;
        3'b100: next_state = Left;
        3'b110: next_state = Left;
        3'b001: next_state = Right;
        3'b011: next_state = Right;
        3'b000: next_state = Back;
        default: next_state = Forward;
        endcase
    
    
    end

endmodule
