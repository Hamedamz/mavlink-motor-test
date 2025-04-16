import argparse
import time
from pymavlink import mavutil


def send_motor_test(mav, motor_index, throttle_type, throttle_value, duration):
    mav.mav.command_long_send(
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST,
        0,                      # confirmation
        motor_index,            # param1: motor index
        throttle_type,          # param2: throttle type
        throttle_value,         # param3: throttle value
        duration,               # param4: timeout (seconds)
        0, 0, 0                 # param5-7: unused
    )


def stop_all_motors(mav, motor_indices):
    print("⚠️ Voltage critical! Stopping all motors...")
    for motor in motor_indices:
        send_motor_test(mav, motor, 2, 0.0, 0)  # Set throttle to 0, duration to 0
    print("✅ Motors stopped.")


def arm_vehicle(mav):
    print("[INFO] Arming vehicle...")
    mav.mav.command_long_send(
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0
    )
    mav.recv_match(type='COMMAND_ACK', blocking=True)
    print("[INFO] Armed.")


def disarm_vehicle(mav):
    print("[INFO] Disarming vehicle...")
    mav.mav.command_long_send(
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0, 0, 0, 0, 0, 0, 0
    )
    mav.recv_match(type='COMMAND_ACK', blocking=True)
    print("[INFO] Disarmed.")


def main():
    parser = argparse.ArgumentParser(description="Motor test script using MAVLink")
    parser.add_argument('--device', type=str, required=True, help='MAVLink device path, e.g., /dev/serial0 or udp:14550')
    parser.add_argument('--baudrate', type=int, default=115200, help='Baud rate for serial connection (default: 115200)')
    parser.add_argument('--motors', type=int, nargs='+', required=True, help='Motor indices to spin (0-based). Example: --motors 0 2 3')
    parser.add_argument('--throttle_type', type=int, choices=[0, 1, 2, 3], default=0, help='Throttle type: 0=Percentage 0-100m, 1=PWM 1000-2000')
    parser.add_argument('--throttle_value', type=float, default=10, help='Throttle value: 0 to 100 (for type 0), 1000-2000 (for type 1)')
    parser.add_argument('--duration', type=int, default=10, help='Test duration in seconds')
    parser.add_argument('--voltage_threshold', type=float, default=10.5, help='Minimum safe voltage to continue test (V)')
    parser.add_argument('--log', type=str, default='log.csv', help='Output log file (CSV)')

    args = parser.parse_args()
    log_path = f"M{'_'.join([str(m) for m in args.motors])}_T{args.throttle_value*100:.0f}_{args.log}"

    print("Connecting to vehicle...")
    mav = mavutil.mavlink_connection(args.device, baud=args.baudrate)
    mav.wait_heartbeat()
    print("Connected. System ID:", mav.target_system)

    # Request parameter list (similar to what QGC does)
    print("Requesting parameters...")
    mav.mav.param_request_list_send(
        mav.target_system, mav.target_component
    )
    
    # Wait briefly for parameters to start coming in
    time.sleep(2)
    
    # Set up data streams - request regular data from the autopilot
    print("Setting up data streams...")
    for i in range(0, 3):  # Try a few times to make sure it gets through
        mav.mav.request_data_stream_send(
            mav.target_system,
            mav.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,  # Request all data
            4,  # 4 Hz
            1   # Start sending
        )
        time.sleep(0.5)
    
    # Give the system time to process the commands and stabilize
    print("Waiting for system initialization...")
    time.sleep(3)

    #arm_vehicle(mav)
    
    print(f"Spinning motors {args.motors} at {args.throttle_value * 100:.0f}% throttle for {args.duration}s each")
    start = time.perf_counter()

    for motor in args.motors:
        send_motor_test(mav, motor, args.throttle_type, args.throttle_value, args.duration)

    with open(log_path, "w") as log:
        log.write("Time(s),Voltage(V),Current(A),RPM\n")

        while True:
            elapsed = time.perf_counter() - start
            if elapsed >= args.duration:
                break

            msg = mav.recv_match(type=['BATTERY_STATUS', 'RPM'], blocking=True, timeout=1)
            voltage = current = rpm = 'N/A'

            if msg:
                if msg.get_type() == 'BATTERY_STATUS':
                    voltage_raw = msg.voltages[0]
                    voltage = voltage_raw / 1000.0 if voltage_raw != 65535 else 'N/A'
                    current = msg.current_battery / 100.0 if msg.current_battery != -1 else 'N/A'

                    if isinstance(voltage, float) and voltage < args.voltage_threshold:
                        stop_all_motors(mav, args.motors)
                        log.write(f"{elapsed:.2f},{voltage},{current},STOPPED\n")
                        print(f"{elapsed:.2f}s | Voltage {voltage:.2f}V dropped below threshold {args.voltage_threshold}V")
                        return

                elif msg.get_type() == 'RPM':
                    rpm = msg.rpm[0]

            print(f"{elapsed:.2f}s | V: {voltage} V | I: {current} A | RPM: {rpm}")
            log.write(f"{elapsed:.2f},{voltage},{current},{rpm}\n")

    print(f"✅ Test complete. Log saved to {log_path}")
    #disarm_vehicle(mav)

if __name__ == "__main__":
    main()
