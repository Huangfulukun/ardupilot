-- thx_ext_bridge.lua
-- Read raw custom frames from scripting serial (SERIAL2), write 16 servos.
-- Frame: 0xA5 0x5A + 16 x uint16 LE + XOR byte
local uart = serial:find_serial(0)
if not uart then
    gcs:send_text(0, "THX bridge: no scripting UART")
    return function() return update, 1000 end
end
	gcs:send_text(6, "THX bridge: uart ok")
uart:begin(115200)

local pwm = {}
for i=0,15 do pwm[i] = 0 end
local last_t = 0
local buf = {}
local state = 0
local idx = 0
local frame = {}
local xor_acc = 0

function update()
    -- Calling available() triggers UARTDriver::_check_connection(), which
    -- accepts the pending TCP client (SITL TCP serial only accepts inside
    -- available()/txspace(); a bare read() never accepts, so no bytes flow).
    uart:available()
    -- drain serial
    while true do
        local b = uart:read()
        if b < 0 then break end
        b = b & 0xFF
        if state == 0 then
            if b == 0xA5 then state = 1; xor_acc = 0xA5 end
        elseif state == 1 then
            if b == 0x5A then state = 2; idx = 0; xor_acc = xor_acc ~ 0x5A
            else state = 0 end
        elseif state == 2 then
            frame[idx] = b
            xor_acc = xor_acc ~ b
            idx = idx + 1
            if idx == 32 then state = 3 end
        elseif state == 3 then
            if b == xor_acc then
                for i=0,15 do
                    pwm[i] = frame[i*2] + frame[i*2+1]*256
                end
                last_t = millis():toint()
            end
            state = 0
        end
    end
    -- write servos
    local now = millis():toint()
    if now - last_t < 200 then
        for i=0,15 do
            local p = pwm[i]
            if p >= 100 and p <= 2200 then
                SRV_Channels:set_output_pwm_chan_timeout(i, p, 150)
            end
        end
    end
    return update, 5
end

	gcs:send_text(6, "THX serial bridge started")
return update()
