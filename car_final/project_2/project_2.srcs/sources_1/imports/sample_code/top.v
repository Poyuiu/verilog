module Top(
    input clk,
    input rst,
    input echo,
    input left_signal,
    input right_signal,
    input mid_signal,
    output trig,
    output reg left_motor,
    output reg [1:0]left,
    output reg right_motor,
    output reg [1:0] right,
    
    output wire left_light,
    output wire mid_light,
    output wire right_light,
    output wire close,
    
    output wire debug_left,
    output wire debug_right
);
    
    assign debug_left = left_motor;
    assign debug_right = right_motor;
    
    wire Rst_n, rst_pb, stop;
    debounce d0(rst_pb, rst, clk);
    onepulse d1(rst_pb, clk, Rst_n);
    
    wire [1:0] mode;
    wire [1:0] pwm;
    
    assign left_light = left_signal;
    assign mid_light = mid_signal;
    assign right_light = right_signal;
    assign close = stop;
    
    parameter Forward = 2'b00;
    parameter Back = 2'b01;
    parameter Left = 2'b10; // turn left "in place"
    parameter Right = 2'b11; // trun right "in place"
    
    motor A(
        .clk(clk),
        .rst(Rst_n),
        .mode(mode),
        .pwm(pwm)
    );

    sonic_top B(
        .clk(clk), 
        .rst(Rst_n), 
        .Echo(echo), 
        .Trig(trig),
        .stop(stop)
    );
    
    tracker_sensor C(
        .clk(clk), 
        .reset(Rst_n), 
        .left_signal(left_signal), 
        .right_signal(right_signal),
        .mid_signal(mid_signal),
        .state(mode)
       );

    always @(*) begin
        // [TO-DO] Use left and right to set your pwm
//        {left_motor, right_motor} = 2'b11;
        if (stop == 1'b1) {left_motor, right_motor} = 2'b00;
        else  {left_motor, right_motor} =  pwm;
    end
    always @* begin
        case (mode)
            Forward: begin
                left = 2'b10;
                right = 2'b10;
            end
            Left: begin
                left = 2'b00;
                right = 2'b10;
            end
            Right: begin
                left = 2'b10;
                right = 2'b00;
            end
            Back: begin
                left = 2'b01;
                right = 2'b01;
            end
        endcase
    end

endmodule

module debounce (pb_debounced, pb, clk);
    output pb_debounced; 
    input pb;
    input clk;
    reg [4:0] DFF;
    
    always @(posedge clk) begin
        DFF[4:1] <= DFF[3:0];
        DFF[0] <= pb; 
    end
    assign pb_debounced = (&(DFF)); 
endmodule

module onepulse (PB_debounced, clk, PB_one_pulse);
    input PB_debounced;
    input clk;
    output reg PB_one_pulse;
    reg PB_debounced_delay;

    always @(posedge clk) begin
        PB_one_pulse <= PB_debounced & (! PB_debounced_delay);
        PB_debounced_delay <= PB_debounced;
    end 
endmodule

