import argparse
from lerobot.motors.motors_bus import FeetechMotorsBus


def main():
    parser = argparse.ArgumentParser(description="Calibrate Feetech motors and store calibration in EEPROM.")
    parser.add_argument('--port', required=True, help='Serial port (e.g., COM8)')
    parser.add_argument('--ids', nargs='+', type=int, required=True, help='Motor IDs (e.g., 1 2 3)')
    parser.add_argument('--model', default='sts3215', help='Motor model (default: sts3215)')
    args = parser.parse_args()

    motors = [
        {'id': id_, 'model': args.model}
        for id_ in args.ids
    ]
    bus = FeetechMotorsBus(port=args.port, motors=motors)

    print("\nMove each motor to its home (zero) position, then press ENTER.")
    input("Press ENTER to continue...")
    print("Writing calibration (homing offsets) to motor EEPROM...")
    bus.set_half_turn_homings()
    print("Calibration written to motor EEPROM. You may now run the FK calibration wizard.")

if __name__ == "__main__":
    main()
