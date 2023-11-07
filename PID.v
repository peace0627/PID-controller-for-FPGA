module PIDController (
  input wire clk, // 时钟信号
  input wire reset, // 复位信号
  input wire [15:0] setpoint, // 期望值
  input wire [15:0] actual_value, // 实际值
  output wire [15:0] control_output // PID控制输出
);

  // PID参数
  parameter [15:0] Kp = 16'b1000; // 比例增益
  parameter [15:0] Ki = 16'b0010; // 积分增益
  parameter [15:0] Kd = 16'b0100; // 微分增益

  reg [15:0] error, integral, derivative, prev_error;

  always @(posedge clk or posedge reset) begin
    if (reset) begin
      error <= 16'b0;
      integral <= 16'b0;
      derivative <= 16'b0;
      prev_error <= 16'b0; // 初始化prev_error
    end else begin
      error <= setpoint - actual_value;

      // 溢出保护
      if (integral + error > 16'h7FFF) integral <= 16'h7FFF;
      else if (integral + error < -16'h8000) integral <= -16'h8000;
      else integral <= integral + error;

      if (derivative + (error - prev_error) > 16'h7FFF) derivative <= 16'h7FFF;
      else if (derivative + (error - prev_error) < -16'h8000) derivative <= -16'h8000;
      else derivative <= derivative + (error - prev_error);

      prev_error <= error; // 更新prev_error
    end
  end

  always @(posedge clk) begin
    if (reset) begin
      control_output <= 16'b0;
    end else begin
      // 溢出保护
      if (Kp * error + Ki * integral + Kd * derivative > 16'h7FFF) control_output <= 16'h7FFF;
      else if (Kp * error + Ki * integral + Kd * derivative < -16'h8000) control_output <= -16'h8000;
      else control_output <= Kp * error + Ki * integral + Kd * derivative;
    end
  end

endmodule
