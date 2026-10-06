float32[3]  position     # base position (m)
float32[3]  orientation  # base orientation (ZYX Euler angle)
float32[12] q            # motor current position（rad）
float32[12] dq           # motor current speed（rad/s）
float32[12] ddq          # motor current acceleration（rad/s^2）
float32[12] footforce    # grf for all feet（N）
float32[12] footpos      # foot position for all feet (m)