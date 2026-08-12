
def calculate_energy_capacity(
    mah: float | int,
    voltage: float | int,
    qty: int
) -> float | int:
    return mah * voltage * 3.6 * qty


if __name__ == "__main__":

    battery_mah = 2600
    battery_voltage = 3.6 
    num_batteries = 8



    print("\nINPUTS:")
    print(f"Battery Capacity (mAh): {battery_mah}")
    print(f"Battery Capacity (J): {battery_voltage}")
    print(f"Number of Batteries: {num_batteries}")

    battery_capacity_joules = calculate_energy_capacity(
        mah=battery_mah, 
        voltage=battery_voltage, 
        qty=num_batteries
    )

    print("\nOUTPUTS:")
    print(f"Battery Capacity (J): {battery_capacity_joules}")
